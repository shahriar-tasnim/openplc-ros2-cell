# Project Status — OpenPLC + ROS 2 (Jazzy) + Gazebo (Harmonic) Pick-and-Place Cell

_Last updated: 2026-09-04_

Target: IEEE ICREST'27, submission 2026-09-30.
Architecture: **PLC decides, ROS executes**, bridged over Modbus TCP (`127.0.0.1:1502`).
Custom 6-DOF arm, IK poses baked in (no runtime ikpy).

---

## Working

- 6-DOF arm moves; `arm_controller` + `joint_state_broadcaster` active.
- Direct `ros2 topic pub /cell/robot_start "{data: true}"` moves the arm — robot chain healthy.
- PLC automatic cycle runs; `SelfTestEnable` forced FALSE at runtime (ST line 70).
- Grasp proven: cube attaches to gripper and rides along.
- Conveyor coil path works end-to-end: `%QX0.0` → Modbus coil 0 → `/cell/conveyor_run`.
- All nodes run: gazebo_robot, conveyor, grasp_manager, part_spawner, modbus_bridge, controllers.

## Current blocker — RobotStart coil not reaching the arm

PLC reaches `CellState = 30` (SEND ROBOT START) but `/cell/robot_start` stays false, so the
robot never fires through the normal cycle.

### What has been ruled OUT (verified by code review, 2026-09-04)

- **Not the ST logic.** State 30 sets `RobotStart := TRUE` as a *held level* (ST line 370),
  cleared only on `xRobotBusy` handshake or 10 s timeout. No one-shot pulse, no late overwrite,
  no gating interlock. All 9 `RobotStart` references checked.
- **Not SelfTest.** `SelfTestEnable := FALSE` is forced first thing every scan (ST line 70),
  overriding the stray `:= true` initializer on the declaration.
- **Not the bridge.** `modbus_bridge_node.py` reads `getValues(1, 0, count=8)` and publishes
  `commands[1]` → `/cell/robot_start`, symmetric with the working conveyor (`commands[0]`).
- **Not the robot node.** `robot_start_callback` is rising-edge triggered; a 0→1 coil transition
  fires it exactly as a direct topic pub does.

### Leading hypothesis — OpenPLC output→coil binding regressed

The path **worked before**, so this is a regression, not a never-configured path. With the ST
using symbolic `AT <SymbolName>` addressing (not `AT %QX0.1`), the actual coil number is bound in
OpenPLC's **Slave Devices** page — exactly the kind of binding that gets dropped/shifted on a
program reload or config edit without touching code.

Most likely causes (in order):
1. Program re-uploaded/recompiled → slave-device point mapping dropped or reordered.
2. Slave Device coil count reduced to 1 → writes ConveyorRun, never transmits RobotStart.
3. VAR block rewritten for the "Milestone 3 self-test" revision → RobotStart moved off coil 1.
4. OpenPLC restart → UI-set mapping not persisted.

### Decisive test (localizes in one shot)

At `CellState = 30`, with OpenPLC monitor showing `RobotStart = TRUE`:

```
read_coils(address=0, count=8, slave=1)   # against the bridge on 127.0.0.1:1502
```

- `bits[1] == False` → OpenPLC-side binding/transport (causes 1–4 above). Fix in OpenPLC.
- `bits[1] == True`  → contradicts the symptom; re-examine the topic/robot side.

### Candidate fix if addressing is the cause

Give the located variables explicit IEC addresses so the coil map is fixed in code, matching the
bridge (`ConveyorRun %QX0.0`, `RobotStart %QX0.1`, `RobotReset %QX0.2`, `MoveHome %QX0.3`; discrete
inputs `%IX0.0..%IX1.3` in the bridge's `feedback[]` order), then verify the Slave Device coil
count is ≥ 4.

---

## Secondary items — already applied in this tree

- `grasp_manager.py`: release line active — `elif (s.startswith("PLACE_AT") or s == "PLACE") and self.holding:`
- `gazebo_robot_node.py`: `from pick_place_cell_controller import grid_place` present
  (no try/except fallback yet — a bare `ros2 run` from a non-installed context could still fail).

## Remaining milestones

1. Fix RobotStart coil path → full 6×6 continuous flow → one-command launch.
2. Experiments: 50+ cycle success rate, cycle-time distribution, Modbus latency, placement
   residual, fault-injection.
3. Optional: object-detection extension (camera → vision node → IK), UR5e branch.
