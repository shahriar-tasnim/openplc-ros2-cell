#!/usr/bin/env python3
"""
extract_metrics_v2.py

Offline ROS 2 bag extractor for the OpenPLC + ROS 2 sorting cell.

Adds two important improvements over v1:
1) Vision diagnostics:
   - counts ALL /cell/detected_label outputs for each active parcel
   - separates pre-pick detections from the PartAtPick->SortDecision window
   - reports matching vs wrong IDs and the IDs actually observed
   - mirrors the sorter's "two matching detections" confirmation rule

2) Conveyor/control timing:
   - active_part -> conveyor_run rising edge
   - conveyor_run rising edge -> PartAtPick rising edge
   - active_part -> PartAtPick
   - PartAtPick -> sort_decision

This lets us distinguish PLC/control waiting time from physical conveyor travel time.
"""

import argparse
import csv
import json
import re
import statistics
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Bool, String


DEFECT_IDS = {3, 9, 17, 22, 28, 34}


def expected_class(parcel_num):
    if parcel_num in DEFECT_IDS:
        return "REJECT"
    if parcel_num is None:
        return "UNKNOWN"
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
    """Return (label, px, py), each possibly None."""
    lid = None
    px = None
    py = None
    for tok in text.split():
        try:
            if tok.startswith("label="):
                lid = int(tok.split("=", 1)[1])
            elif tok.startswith("px="):
                px = float(tok.split("=", 1)[1])
            elif tok.startswith("py="):
                py = float(tok.split("=", 1)[1])
        except ValueError:
            pass
    return lid, px, py


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


def unique_in_order(values):
    seen = set()
    out = []
    for v in values:
        if v not in seen:
            seen.add(v)
            out.append(v)
    return out


def mean_or_none(values):
    vals = [v for v in values if v is not None]
    return statistics.mean(vals) if vals else None


def median_or_none(values):
    vals = [v for v in values if v is not None]
    return statistics.median(vals) if vals else None


def main():
    ap = argparse.ArgumentParser(
        description="Extract parcel-level metrics and vision diagnostics from the sorting-cell rosbag."
    )
    ap.add_argument(
        "bag",
        help="Bag directory, e.g. experiment_data/baseline_trial_002",
    )
    ap.add_argument("--expected-parcels", type=int, default=36)
    ap.add_argument(
        "--output",
        default=None,
        help="Output CSV path (default: BAG/parcels_v2.csv)",
    )
    args = ap.parse_args()

    bag = Path(args.bag)
    if not bag.exists():
        raise SystemExit(f"Bag directory does not exist: {bag}")

    output_csv = Path(args.output) if args.output else bag / "parcels_v2.csv"
    summary_json = output_csv.with_name("run_summary_v2.json")

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
    robot_status_counts = Counter()

    # Global state is required because these Bool topics are periodically republished.
    bool_topics = {
        "/cell/conveyor_run",
        "/cell/part_at_pick",
        "/cell/robot_start",
        "/cell/robot_busy",
        "/cell/robot_done",
        "/cell/robot_fault",
    }
    prev_bool = {topic: False for topic in bool_topics}

    cycle_mode = None  # "valid", "duplicate", or None

    def new_record(part, t_ns):
        n = parcel_number(part)
        rec = {
            "cycle": len(records) + 1,
            "parcel_id": part,
            "parcel_num": n,
            "expected_class": expected_class(n),

            "t_active": t_ns,
            "t_conveyor_run_rise": None,
            "t_part_at_pick": None,

            # Every detector output associated with this active parcel.
            # Each entry: (time_ns, label, px, py)
            "detections": [],

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
            if current is None:
                continue

            # Keep detections until this unique parcel's first valid DONE.
            if current["t_done"] is not None:
                continue

            msg = deserialize_message(data, String)
            lid, px, py = parse_detected_label(msg.data)

            if lid is not None:
                current["detections"].append((t_ns, lid, px, py))
            continue

        if topic == "/cell/sort_decision":
            if current is None or current["t_done"] is not None:
                continue

            msg = deserialize_message(data, String)
            label, dest = parse_sort_decision(msg.data)

            # /cell/sort_decision is republished until RobotBusy.
            # Only the first logical decision belongs in the metrics.
            if current["t_sort_decision"] is None:
                current["sort_label"] = label
                current["commanded_destination"] = dest
                current["t_sort_decision"] = t_ns
            continue

        if topic == "/cell/robot_status":
            msg = deserialize_message(data, String)
            status = msg.data.strip()
            robot_status_counts[status] += 1

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

        if topic in bool_topics:
            msg = deserialize_message(data, Bool)
            value = bool(msg.data)
            rising = value and not prev_bool[topic]
            prev_bool[topic] = value

            if current is None:
                continue

            # Ignore periodic/repeated Bool messages after the parcel is complete,
            # except global state above is still updated.
            if current["t_done"] is not None:
                continue

            if topic == "/cell/conveyor_run":
                # First PLC ConveyorRun rising edge associated with this parcel.
                if rising and current["t_conveyor_run_rise"] is None:
                    current["t_conveyor_run_rise"] = t_ns

            elif topic == "/cell/part_at_pick":
                if rising and current["t_part_at_pick"] is None:
                    current["t_part_at_pick"] = t_ns

            elif topic == "/cell/robot_start":
                if rising and current["t_robot_start"] is None:
                    current["t_robot_start"] = t_ns

            elif topic == "/cell/robot_fault":
                if value:
                    current["fault"] = 1
                    if current["t_fault"] is None:
                        current["t_fault"] = t_ns

    completed = [r for r in records if r["t_done"] is not None]
    accepted = completed[:args.expected_parcels]

    first_active_ns = accepted[0]["t_active"] if accepted else None

    def rel_s(t_ns):
        if t_ns is None or first_active_ns is None:
            return None
        return (t_ns - first_active_ns) / 1e9

    fieldnames = [
        "cycle",
        "parcel_id",
        "parcel_num",
        "expected_class",
        "sort_label",
        "commanded_destination",
        "correct_command",
        "fault",

        # Vision diagnostics
        "vision_confirmed",
        "decision_source_inferred",
        "detections_active_total",
        "detections_pre_pick_total",
        "detections_pick_window_total",
        "matching_detections_pick_window",
        "wrong_detections_pick_window",
        "observed_ids_pre_pick",
        "observed_ids_pick_window",
        "first_id_pick_window",
        "first_matching_detection_latency_s",
        "second_matching_detection_latency_s",

        # Absolute experiment-relative event times
        "t_active_s",
        "t_conveyor_run_rise_s",
        "t_part_at_pick_s",
        "t_sort_decision_s",
        "t_robot_start_s",
        "t_busy_s",
        "t_pick_s",
        "t_lift_s",
        "t_place_at_s",
        "t_pre_pick_s",
        "t_done_s",

        # Corrected control/conveyor timing decomposition
        "active_to_conveyor_run_s",
        "conveyor_run_to_part_at_pick_s",
        "active_to_part_at_pick_s",
        "part_at_pick_to_sort_decision_s",

        # Control/robot timing
        "part_to_robot_start_s",
        "sort_to_busy_s",
        "robot_start_to_busy_s",
        "pick_approach_s",
        "lift_motion_s",
        "place_transfer_s",
        "return_s",
        "robot_motion_s",
        "busy_to_done_s",
        "total_cycle_s",
    ]

    rows = []

    for rec in accepted:
        t_pick_sensor = rec["t_part_at_pick"]
        t_decision = rec["t_sort_decision"]
        expected_id = rec["parcel_num"]

        all_dets = rec["detections"]

        pre_pick = []
        pick_window = []

        for det in all_dets:
            t_det, lid, px, py = det

            if t_pick_sensor is not None and t_det < t_pick_sensor:
                pre_pick.append(det)

            # Match the sorter's actual decision window:
            # PartAtPick TRUE until the FIRST logical sort decision.
            if (
                t_pick_sensor is not None
                and t_det >= t_pick_sensor
                and (t_decision is None or t_det <= t_decision)
            ):
                pick_window.append(det)

        matching = [d for d in pick_window if expected_id is not None and d[1] == expected_id]
        wrong = [d for d in pick_window if expected_id is not None and d[1] != expected_id]

        # sorter.py ignores wrong IDs before counting pending hits, so two matching
        # outputs anywhere in the decision window are sufficient to mirror its rule.
        vision_confirmed = int(len(matching) >= 2)

        if vision_confirmed:
            source = "vision"
        elif len(matching) == 1:
            source = "fallback_after_one_matching_detection"
        elif len(pick_window) > 0:
            source = "fallback_wrong_id_seen"
        else:
            source = "fallback_no_detector_output"

        first_matching = matching[0][0] if len(matching) >= 1 else None
        second_matching = matching[1][0] if len(matching) >= 2 else None

        pre_ids = unique_in_order([d[1] for d in pre_pick])
        pick_ids = unique_in_order([d[1] for d in pick_window])
        first_pick_id = pick_window[0][1] if pick_window else None

        correct = int(
            rec["commanded_destination"] is not None
            and rec["commanded_destination"] == rec["expected_class"]
        )

        row = {
            "cycle": rec["cycle"],
            "parcel_id": rec["parcel_id"],
            "parcel_num": rec["parcel_num"],
            "expected_class": rec["expected_class"],
            "sort_label": rec["sort_label"],
            "commanded_destination": rec["commanded_destination"],
            "correct_command": correct,
            "fault": rec["fault"],

            "vision_confirmed": vision_confirmed,
            "decision_source_inferred": source,
            "detections_active_total": len(all_dets),
            "detections_pre_pick_total": len(pre_pick),
            "detections_pick_window_total": len(pick_window),
            "matching_detections_pick_window": len(matching),
            "wrong_detections_pick_window": len(wrong),
            "observed_ids_pre_pick": "|".join(map(str, pre_ids)),
            "observed_ids_pick_window": "|".join(map(str, pick_ids)),
            "first_id_pick_window": "" if first_pick_id is None else first_pick_id,
            "first_matching_detection_latency_s": fmt(
                seconds_between(t_pick_sensor, first_matching)
            ),
            "second_matching_detection_latency_s": fmt(
                seconds_between(t_pick_sensor, second_matching)
            ),

            "t_active_s": fmt(rel_s(rec["t_active"])),
            "t_conveyor_run_rise_s": fmt(rel_s(rec["t_conveyor_run_rise"])),
            "t_part_at_pick_s": fmt(rel_s(rec["t_part_at_pick"])),
            "t_sort_decision_s": fmt(rel_s(rec["t_sort_decision"])),
            "t_robot_start_s": fmt(rel_s(rec["t_robot_start"])),
            "t_busy_s": fmt(rel_s(rec["t_busy"])),
            "t_pick_s": fmt(rel_s(rec["t_pick"])),
            "t_lift_s": fmt(rel_s(rec["t_lift"])),
            "t_place_at_s": fmt(rel_s(rec["t_place_at"])),
            "t_pre_pick_s": fmt(rel_s(rec["t_pre_pick"])),
            "t_done_s": fmt(rel_s(rec["t_done"])),

            "active_to_conveyor_run_s": fmt(
                seconds_between(rec["t_active"], rec["t_conveyor_run_rise"])
            ),
            "conveyor_run_to_part_at_pick_s": fmt(
                seconds_between(rec["t_conveyor_run_rise"], rec["t_part_at_pick"])
            ),
            "active_to_part_at_pick_s": fmt(
                seconds_between(rec["t_active"], rec["t_part_at_pick"])
            ),
            "part_at_pick_to_sort_decision_s": fmt(
                seconds_between(rec["t_part_at_pick"], rec["t_sort_decision"])
            ),

            "part_to_robot_start_s": fmt(
                seconds_between(rec["t_part_at_pick"], rec["t_robot_start"])
            ),
            "sort_to_busy_s": fmt(
                seconds_between(rec["t_sort_decision"], rec["t_busy"])
            ),
            "robot_start_to_busy_s": fmt(
                seconds_between(rec["t_robot_start"], rec["t_busy"])
            ),
            "pick_approach_s": fmt(
                seconds_between(rec["t_pick"], rec["t_lift"])
            ),
            "lift_motion_s": fmt(
                seconds_between(rec["t_lift"], rec["t_place_at"])
            ),
            "place_transfer_s": fmt(
                seconds_between(rec["t_place_at"], rec["t_pre_pick"])
            ),
            "return_s": fmt(
                seconds_between(rec["t_pre_pick"], rec["t_done"])
            ),
            "robot_motion_s": fmt(
                seconds_between(rec["t_pick"], rec["t_done"])
            ),
            "busy_to_done_s": fmt(
                seconds_between(rec["t_busy"], rec["t_done"])
            ),
            "total_cycle_s": fmt(
                seconds_between(rec["t_active"], rec["t_done"])
            ),
        }

        # Keep raw values privately for summary calculations.
        row["_raw_active_to_conveyor"] = seconds_between(
            rec["t_active"], rec["t_conveyor_run_rise"]
        )
        row["_raw_conveyor_motion"] = seconds_between(
            rec["t_conveyor_run_rise"], rec["t_part_at_pick"]
        )
        row["_raw_pick_to_decision"] = seconds_between(
            rec["t_part_at_pick"], rec["t_sort_decision"]
        )
        row["_raw_busy_to_done"] = seconds_between(
            rec["t_busy"], rec["t_done"]
        )
        rows.append(row)

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})

    vision_confirmed_count = sum(r["vision_confirmed"] for r in rows)
    correct_count = sum(r["correct_command"] for r in rows)

    fallback_reasons = Counter(
        r["decision_source_inferred"]
        for r in rows
        if r["decision_source_inferred"] != "vision"
    )

    parcels_with_any_pick_detection = sum(
        1 for r in rows if r["detections_pick_window_total"] > 0
    )
    parcels_with_any_matching_pick_detection = sum(
        1 for r in rows if r["matching_detections_pick_window"] > 0
    )

    run_start = accepted[0]["t_active"] if accepted else None
    run_end = accepted[-1]["t_done"] if accepted else None
    run_duration_s = seconds_between(run_start, run_end)

    summary = {
        "bag": str(bag),
        "raw_active_part_messages": raw_active_messages,
        "unique_active_parts": len(by_part),
        "repeated_active_part_messages": repeated_active_messages,
        "completed_unique_cycles": len(completed),
        "accepted_cycles": len(accepted),
        "expected_parcels": args.expected_parcels,
        "extra_robot_cycles_excluded": extra_robot_cycles,
        "faulted_parcels": sum(r["fault"] for r in rows),

        "correct_commanded_destinations": correct_count,
        "command_accuracy": correct_count / len(rows) if rows else None,

        "vision_confirmed_parcels": vision_confirmed_count,
        "vision_confirmation_rate": (
            vision_confirmed_count / len(rows) if rows else None
        ),
        "parcels_with_any_detector_output_in_pick_window": parcels_with_any_pick_detection,
        "parcels_with_any_matching_detector_output_in_pick_window": (
            parcels_with_any_matching_pick_detection
        ),
        "fallback_reason_counts": dict(fallback_reasons),

        "run_duration_s": run_duration_s,
        "throughput_parcels_per_min": (
            len(rows) / run_duration_s * 60.0
            if run_duration_s and run_duration_s > 0
            else None
        ),

        "active_to_conveyor_run_mean_s": mean_or_none(
            [r["_raw_active_to_conveyor"] for r in rows]
        ),
        "active_to_conveyor_run_median_s": median_or_none(
            [r["_raw_active_to_conveyor"] for r in rows]
        ),
        "conveyor_run_to_part_at_pick_mean_s": mean_or_none(
            [r["_raw_conveyor_motion"] for r in rows]
        ),
        "conveyor_run_to_part_at_pick_median_s": median_or_none(
            [r["_raw_conveyor_motion"] for r in rows]
        ),
        "part_at_pick_to_sort_decision_mean_s": mean_or_none(
            [r["_raw_pick_to_decision"] for r in rows]
        ),
        "part_at_pick_to_sort_decision_median_s": median_or_none(
            [r["_raw_pick_to_decision"] for r in rows]
        ),
        "busy_to_done_mean_s": mean_or_none(
            [r["_raw_busy_to_done"] for r in rows]
        ),
        "busy_to_done_median_s": median_or_none(
            [r["_raw_busy_to_done"] for r in rows]
        ),
        "robot_status_counts": dict(robot_status_counts),
    }

    with summary_json.open("w") as f:
        json.dump(summary, f, indent=2)

    print("\n===== EXTRACTION V2 COMPLETE =====")
    print(f"Bag                         : {bag}")
    print(f"Unique parcels introduced  : {len(by_part)}")
    print(f"Completed unique cycles     : {len(completed)}")
    print(f"Accepted cycles             : {len(rows)}/{args.expected_parcels}")
    print(f"Extra robot cycles excluded : {extra_robot_cycles}")
    print(f"Faulted parcels             : {summary['faulted_parcels']}")
    print(f"Commanded correctly         : {correct_count}/{len(rows)}")
    print()
    print("----- VISION DIAGNOSTIC -----")
    print(f"Vision confirmed (>=2 match): {vision_confirmed_count}/{len(rows)}")
    print(
        "Any detector output at pick : "
        f"{parcels_with_any_pick_detection}/{len(rows)}"
    )
    print(
        "Any matching ID at pick      : "
        f"{parcels_with_any_matching_pick_detection}/{len(rows)}"
    )
    print(f"Fallback reasons             : {dict(fallback_reasons)}")
    print()
    print("----- CONTROL / CONVEYOR TIMING -----")

    def show_metric(label, mean_key, median_key):
        mean_v = summary[mean_key]
        med_v = summary[median_key]
        if mean_v is None:
            print(f"{label:29s}: n/a")
        else:
            print(f"{label:29s}: mean={mean_v:.3f} s | median={med_v:.3f} s")

    show_metric(
        "Active -> ConveyorRun rise",
        "active_to_conveyor_run_mean_s",
        "active_to_conveyor_run_median_s",
    )
    show_metric(
        "ConveyorRun rise -> Pick",
        "conveyor_run_to_part_at_pick_mean_s",
        "conveyor_run_to_part_at_pick_median_s",
    )
    show_metric(
        "PartAtPick -> Decision",
        "part_at_pick_to_sort_decision_mean_s",
        "part_at_pick_to_sort_decision_median_s",
    )
    show_metric(
        "Robot BUSY -> DONE",
        "busy_to_done_mean_s",
        "busy_to_done_median_s",
    )

    if run_duration_s is not None:
        print(f"\nExperiment duration          : {run_duration_s:.3f} s")
        print(
            "Raw-window throughput        : "
            f"{summary['throughput_parcels_per_min']:.3f} parcels/min"
        )

    print(f"\nCSV written to              : {output_csv}")
    print(f"Summary written to          : {summary_json}")

    if len(rows) != args.expected_parcels:
        print(
            "\nWARNING: run does not contain the expected number "
            "of complete unique cycles."
        )

    print("\nVISION INTERPRETATION:")
    print(
        "  no detector output at pick -> camera/detector produced nothing "
        "during the 1 s decision window"
    )
    print(
        "  detector output but no matching ID -> detector selected another "
        "visible marker; check pick-pixel calibration / scene clutter"
    )
    print(
        "  exactly one matching ID -> detector saw the active marker, but not "
        "twice before the 1 s fallback"
    )
    print(
        "  >=2 matching IDs -> sorter should be able to confirm by vision"
    )


if __name__ == "__main__":
    main()
