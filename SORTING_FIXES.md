# Smooth ID-based parcel sorting fix

## Restored parcel appearance
- Restored `cell_world.sdf` from the pre-`make_unlabelled.py` backup.
- All 36 parcels again use the same orange body material.
- All 36 parcels carry their ArUco labels.
- Removed `make_unlabelled.py`, which had removed labels from IDs 3, 9, 17, 22, 28, 34 and tinted those parcels grey.

## Four ID-based classes
The six parcels previously used as visual/unlabelled defects are now defects by ID only:

- **DEFECT / REJECT:** 3, 9, 17, 22, 28, 34
- **A:** 0..11 excluding defect IDs
- **B:** 12..23 excluding defect IDs
- **C:** 24..35 excluding defect IDs

This keeps the same 36 unique ArUco IDs while making every parcel look physically the same.

## Synchronization fixes
1. Removed the duplicate `part_spawner` launch entry. The previous launch started **two feeder nodes**, which could publish different active parcels and break synchronization.
2. `label_detector.py` now checks every visible marker and publishes the marker nearest the calibrated pick point instead of blindly using OpenCV's first returned marker.
3. `sorter.py` requires two consistent nearest-marker frames, then publishes a combined `/cell/sort_decision` event (`label=<id> dest=<station>`) for the parcel currently at the pick station.
4. `gazebo_robot_node.py` treats PLC `RobotStart` as a pending request until that current sort decision exists. This removes the race where the robot could latch the previous destination or `REJECT` before vision finished.
5. The sort decision is consumed when the cycle completes, so every next parcel requires a fresh ArUco decision.
6. `grasp_manager.py` listens to the same combined decision event as the robot, so parcel motion and arm destination use the same latched route.
7. `part_spawner.py` still shuffles all 36 parcels and advances only on a complete RobotBusy -> RobotDone cycle.

## Expected sequence

`random parcel -> conveyor -> PartAtPick -> ArUco decision -> PLC RobotStart -> RobotBusy -> PICK -> LIFT -> selected pallet/reject -> PRE_PICK -> RobotDone -> next random parcel`

The existing Modbus address mapping is unchanged.
