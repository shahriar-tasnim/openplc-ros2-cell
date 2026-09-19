#!/usr/bin/env python3
"""
make_unlabelled.py -- strip the ArUco label from some parcels so the cell has
something to reject.

Parcels listed in UNLABELLED get their label plate removed from the world, so
the camera finds no marker, the sorter cannot classify them, and they are
routed to the reject bin. This demonstrates the "no readable label -> reject"
case that the sorting logic already supports.

Run:  python3 make_unlabelled.py
"""
import os, re, shutil

WORLD = os.path.expanduser(
    "~/openplc_ros2_cell_ws/src/pick_place_cell_gazebo/worlds/cell_world.sdf")

# parcels that will carry NO label (spread across the range so they turn up
# at different points in the shuffled feed)
UNLABELLED = [3, 9, 17, 22, 28, 34]


def main():
    w = open(WORLD).read()
    shutil.copy(WORLD, WORLD + ".bak_unlabelled")

    stripped = []
    for n in UNLABELLED:
        # find that cube's model block
        m = re.search(rf'(    <model name="cube_{n}">.*?</model>\n)', w, re.DOTALL)
        if not m:
            print(f"  cube_{n}: not found, skipped")
            continue
        block = m.group(1)
        # remove the <visual name="label"> ... </visual> plate from it
        new_block, count = re.subn(
            r'\s*<visual name="label">.*?</visual>', '', block, flags=re.DOTALL)
        if count == 0:
            print(f"  cube_{n}: no label plate, already plain")
            continue
        # tint it slightly differently so unlabelled parcels are easy to spot
        new_block = new_block.replace(
            "<diffuse>0.85 0.35 0.1 1</diffuse>",
            "<diffuse>0.55 0.55 0.55 1</diffuse>")
        w = w.replace(block, new_block)
        stripped.append(n)

    open(WORLD, "w").write(w)
    print(f"removed labels from {len(stripped)} parcels: {stripped}")
    print("these are tinted grey and will be routed to REJECT")
    print(f"backup: {WORLD}.bak_unlabelled")


if __name__ == "__main__":
    main()
