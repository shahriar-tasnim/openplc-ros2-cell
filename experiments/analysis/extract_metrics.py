#!/usr/bin/env python3
import argparse
import csv
import json
import re
import statistics
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool, String

DEFECT_IDS = {3, 9, 17, 22, 28, 34}


def expected_class(parcel_num):
    if parcel_num in DEFECT_IDS:
        return "REJECT"
    if 0 <= parcel_num <= 11:
        return "A"
    if 12 <= parcel_num <= 23:
        return "B"
    if 24 <= parcel_num <= 35:
        return "C"
    return "UNKNOWN"


def parcel_number(name):
    m = re.fullmatch(r"cube_(\d+)", name.strip())
    return int(m.group(1)) if m else None


def parse_detected_label(text):
    m = re.search(r"(?:^|\s)label=(-?\d+)(?:\s|$)", text)
    return int(m.group(1)) if m else None


def parse_sort_decision(text):
    label = None
    dest = None
    for tok in text.split():
        if tok.startswith("label="):
            try:
                label = int(tok.split("=", 1)[1])
            except ValueError:
                pass
        elif tok.startswith("dest="):
            dest = tok.split("=", 1)[1]
    return label, dest


def seconds_between(a, b):
    if a is None or b is None:
        return None
    return (b - a) / 1e9


def fmt(v):
    return "" if v is None else f"{v:.6f}"


def main():
    ap = argparse.ArgumentParser(
        description="Extract parcel-level metrics from the OpenPLC/ROS2 sorting-cell rosbag."
    )
    ap.add_argument("bag", help="Bag directory, e.g. experiment_data/baseline_trial_002")
    ap.add_argument("--expected-parcels", type=int, default=36)
    ap.add_argument("--output", default=None,
                    help="Output CSV path (default: BAG/parcels.csv)")
    args = ap.parse_args()

    bag = Path(args.bag)
    if not bag.exists():
        raise SystemExit(f"Bag directory does not exist: {bag}")

    output_csv = Path(args.output) if args.output else bag / "parcels.csv"
    summary_json = output_csv.with_name("run_summary.json")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    records = []
    by_part = {}
    current = None

    raw_active_messages = 0
    repeated_active_messages = 0
    extra_robot_cycles = 0
    robot_status_counts = {}

    prev_bool = {
        "/cell/part_at_pick": False,
        "/cell/robot_start": False,
        "/cell/robot_busy": False,
        "/cell/robot_done": False,
        "/cell/robot_fault": False,
    }

    cycle_mode = None  # "valid", "duplicate", or None

    def new_record(part, t_ns):
        n = parcel_number(part)
        rec = {
            "cycle": len(records) + 1,
            "parcel_id": part,
            "parcel_num": n,
            "expected_class": expected_class(n) if n is not None else "UNKNOWN",
            "t_active": t_ns,
            "t_part_at_pick": None,
            "matching_detection_times": [],
            "sort_label": None,
            "commanded_destination": None,
            "t_sort_decision": None,
            "t_robot_start": None,
            "t_busy": None,
            "t_pick": None,
            "t_lift": None,
            "t_place_at": None,
            "t_pre_pick": None,
            "t_done": None,
            "fault": 0,
            "t_fault": None,
        }
        records.append(rec)
        by_part[part] = rec
        return rec

    while reader.has_next():
        topic, data, t_ns = reader.read_next()

        if topic == "/cell/active_part":
            msg = deserialize_message(data, String)
            raw_active_messages += 1
            part = msg.data.strip()
            if part in by_part:
                repeated_active_messages += 1
                current = by_part[part]
            else:
                current = new_record(part, t_ns)
            cycle_mode = None
            continue

        if topic == "/cell/detected_label":
            if current is None or current["t_done"] is not None:
                continue
            msg = deserialize_message(data, String)
            lid = parse_detected_label(msg.data)
            if lid is None or current["parcel_num"] is None:
                continue
            if current["t_part_at_pick"] is None or current["t_sort_decision"] is not None:
                continue
            if lid == current["parcel_num"]:
                current["matching_detection_times"].append(t_ns)
            continue

        if topic == "/cell/sort_decision":
            if current is None or current["t_done"] is not None:
                continue
            msg = deserialize_message(data, String)
            label, dest = parse_sort_decision(msg.data)
            if current["t_sort_decision"] is None:
                current["sort_label"] = label
                current["commanded_destination"] = dest
                current["t_sort_decision"] = t_ns
            continue

        if topic == "/cell/robot_status":
            msg = deserialize_message(data, String)
            status = msg.data.strip()
            robot_status_counts[status] = robot_status_counts.get(status, 0) + 1

            if status == "BUSY":
                if current is None:
                    cycle_mode = None
                elif current["t_done"] is None and current["t_busy"] is None:
                    current["t_busy"] = t_ns
                    cycle_mode = "valid"
                else:
                    cycle_mode = "duplicate"
                continue

            if cycle_mode == "valid" and current is not None:
                if status == "PICK" and current["t_pick"] is None:
                    current["t_pick"] = t_ns
                elif status == "LIFT" and current["t_lift"] is None:
                    current["t_lift"] = t_ns
                elif status == "PLACE_AT" and current["t_place_at"] is None:
                    current["t_place_at"] = t_ns
                elif status == "PRE_PICK" and current["t_pre_pick"] is None:
                    current["t_pre_pick"] = t_ns
                elif status == "DONE" and current["t_done"] is None:
                    current["t_done"] = t_ns
                    cycle_mode = None

            elif cycle_mode == "duplicate" and status == "DONE":
                extra_robot_cycles += 1
                cycle_mode = None

            continue

        if topic in prev_bool:
            msg = deserialize_message(data, Bool)
            value = bool(msg.data)
            rising = value and not prev_bool[topic]
            prev_bool[topic] = value

            if current is None:
                continue

            if topic == "/cell/part_at_pick":
                if rising and current["t_done"] is None and current["t_part_at_pick"] is None:
                    current["t_part_at_pick"] = t_ns

            elif topic == "/cell/robot_start":
                if rising and current["t_done"] is None and current["t_robot_start"] is None:
                    current["t_robot_start"] = t_ns

            elif topic == "/cell/robot_fault":
                if value and current["t_done"] is None:
                    current["fault"] = 1
                    if current["t_fault"] is None:
                        current["t_fault"] = t_ns

    completed = [r for r in records if r["t_done"] is not None]
    first_active_ns = records[0]["t_active"] if records else None
    accepted = completed[:args.expected_parcels]

    def rel_s(t_ns):
        if t_ns is None or first_active_ns is None:
            return None
        return (t_ns - first_active_ns) / 1e9

    fieldnames = [
        "cycle", "parcel_id", "parcel_num", "expected_class",
        "sort_label", "commanded_destination", "correct_command",
        "vision_confirmed", "matching_detection_count",
        "decision_source_inferred", "fault",
        "t_active_s", "t_part_at_pick_s",
        "t_first_matching_detection_s", "t_second_matching_detection_s",
        "t_sort_decision_s", "t_robot_start_s", "t_busy_s",
        "t_pick_s", "t_lift_s", "t_place_at_s", "t_pre_pick_s", "t_done_s",
        "conveyor_s", "first_detection_latency_s", "vision_decision_latency_s",
        "part_to_robot_start_s", "sort_to_busy_s", "robot_start_to_busy_s",
        "pick_approach_s", "lift_motion_s", "place_transfer_s", "return_s",
        "robot_motion_s", "busy_to_done_s", "total_cycle_s",
    ]

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for rec in accepted:
            detections = rec["matching_detection_times"]
            first_det = detections[0] if len(detections) >= 1 else None
            second_det = detections[1] if len(detections) >= 2 else None

            # sorter.py requires two matching detections before a vision decision.
            vision_confirmed = int(len(detections) >= 2)
            source = "vision" if vision_confirmed else "fallback_or_unconfirmed"

            correct = int(
                rec["commanded_destination"] is not None
                and rec["commanded_destination"] == rec["expected_class"]
            )

            writer.writerow({
                "cycle": rec["cycle"],
                "parcel_id": rec["parcel_id"],
                "parcel_num": rec["parcel_num"],
                "expected_class": rec["expected_class"],
                "sort_label": rec["sort_label"],
                "commanded_destination": rec["commanded_destination"],
                "correct_command": correct,
                "vision_confirmed": vision_confirmed,
                "matching_detection_count": len(detections),
                "decision_source_inferred": source,
                "fault": rec["fault"],
                "t_active_s": fmt(rel_s(rec["t_active"])),
                "t_part_at_pick_s": fmt(rel_s(rec["t_part_at_pick"])),
                "t_first_matching_detection_s": fmt(rel_s(first_det)),
                "t_second_matching_detection_s": fmt(rel_s(second_det)),
                "t_sort_decision_s": fmt(rel_s(rec["t_sort_decision"])),
                "t_robot_start_s": fmt(rel_s(rec["t_robot_start"])),
                "t_busy_s": fmt(rel_s(rec["t_busy"])),
                "t_pick_s": fmt(rel_s(rec["t_pick"])),
                "t_lift_s": fmt(rel_s(rec["t_lift"])),
                "t_place_at_s": fmt(rel_s(rec["t_place_at"])),
                "t_pre_pick_s": fmt(rel_s(rec["t_pre_pick"])),
                "t_done_s": fmt(rel_s(rec["t_done"])),
                "conveyor_s": fmt(seconds_between(rec["t_active"], rec["t_part_at_pick"])),
                "first_detection_latency_s": fmt(seconds_between(rec["t_part_at_pick"], first_det)),
                "vision_decision_latency_s": fmt(seconds_between(rec["t_part_at_pick"], rec["t_sort_decision"])),
                "part_to_robot_start_s": fmt(seconds_between(rec["t_part_at_pick"], rec["t_robot_start"])),
                "sort_to_busy_s": fmt(seconds_between(rec["t_sort_decision"], rec["t_busy"])),
                "robot_start_to_busy_s": fmt(seconds_between(rec["t_robot_start"], rec["t_busy"])),
                "pick_approach_s": fmt(seconds_between(rec["t_pick"], rec["t_lift"])),
                "lift_motion_s": fmt(seconds_between(rec["t_lift"], rec["t_place_at"])),
                "place_transfer_s": fmt(seconds_between(rec["t_place_at"], rec["t_pre_pick"])),
                "return_s": fmt(seconds_between(rec["t_pre_pick"], rec["t_done"])),
                "robot_motion_s": fmt(seconds_between(rec["t_pick"], rec["t_done"])),
                "busy_to_done_s": fmt(seconds_between(rec["t_busy"], rec["t_done"])),
                "total_cycle_s": fmt(seconds_between(rec["t_active"], rec["t_done"])),
            })

    run_start = accepted[0]["t_active"] if accepted else None
    run_end = accepted[-1]["t_done"] if accepted else None
    run_duration_s = seconds_between(run_start, run_end)

    busy_durations = [
        seconds_between(r["t_busy"], r["t_done"])
        for r in accepted
        if r["t_busy"] is not None and r["t_done"] is not None
    ]

    vision_count = sum(
        1 for r in accepted if len(r["matching_detection_times"]) >= 2
    )
    correct_count = sum(
        1 for r in accepted
        if r["commanded_destination"] == r["expected_class"]
    )

    summary = {
        "bag": str(bag),
        "raw_active_part_messages": raw_active_messages,
        "unique_active_parts": len(by_part),
        "repeated_active_part_messages": repeated_active_messages,
        "completed_unique_cycles": len(completed),
        "accepted_cycles": len(accepted),
        "expected_parcels": args.expected_parcels,
        "extra_robot_cycles_excluded": extra_robot_cycles,
        "faulted_parcels": sum(r["fault"] for r in accepted),
        "vision_confirmed_parcels": vision_count,
        "vision_confirmation_rate": (
            vision_count / len(accepted) if accepted else None
        ),
        "correct_commanded_destinations": correct_count,
        "command_accuracy": (
            correct_count / len(accepted) if accepted else None
        ),
        "run_duration_s": run_duration_s,
        "throughput_parcels_per_min": (
            len(accepted) / run_duration_s * 60.0
            if run_duration_s and run_duration_s > 0 else None
        ),
        "busy_to_done_mean_s": (
            statistics.mean(busy_durations) if busy_durations else None
        ),
        "busy_to_done_median_s": (
            statistics.median(busy_durations) if busy_durations else None
        ),
        "robot_status_counts": robot_status_counts,
    }

    with summary_json.open("w") as f:
        json.dump(summary, f, indent=2)

    print("\n===== EXTRACTION COMPLETE =====")
    print(f"Bag                      : {bag}")
    print(f"Unique parcels introduced: {len(by_part)}")
    print(f"Completed unique cycles  : {len(completed)}")
    print(f"Accepted cycles          : {len(accepted)}/{args.expected_parcels}")
    print(f"Extra robot cycles       : {extra_robot_cycles} (excluded)")
    print(f"Faulted parcels          : {summary['faulted_parcels']}")
    print(f"Vision confirmed         : {vision_count}/{len(accepted)}")
    print(f"Commanded correctly      : {correct_count}/{len(accepted)}")

    if run_duration_s is not None:
        print(f"Experiment duration      : {run_duration_s:.3f} s")
        print(f"Throughput               : {summary['throughput_parcels_per_min']:.3f} parcels/min")

    if busy_durations:
        print(f"Robot BUSY->DONE mean    : {summary['busy_to_done_mean_s']:.3f} s")
        print(f"Robot BUSY->DONE median  : {summary['busy_to_done_median_s']:.3f} s")

    print(f"\nCSV written to           : {output_csv}")
    print(f"Summary written to       : {summary_json}")

    if len(accepted) != args.expected_parcels:
        print("\nWARNING: run does not contain the expected number of complete unique cycles.")


if __name__ == "__main__":
    main()
