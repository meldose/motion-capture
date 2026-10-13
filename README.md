# Motion Studio to G1 Dex3: offline preparation

`export_dex3_candidate.py` turns the cached Motion Studio hand-bone quaternions into a 20 Hz normalized gesture recording. It does **not** connect to ROS/DDS or command the robot. The source capture is timed at about 120.7 Hz (3,259 frames in ~27 seconds), and the converter preserves that timing.

The recording contains these source bones: thumb, index, and middle finger for both hands. Dex3 has seven motors per hand in this order (the same order used by `HandCmd.motor_cmd` and `HandState.motor_state`):

`thumb_0, thumb_1, thumb_2, index_0, index_1, middle_0, middle_1`

Run from this directory:

```bash
python3 export_dex3_candidate.py ../.ms/cache/algo/6kl2x8slyswzguvw3njghlpj52tcqtlk_12691235622399232648 dex3_gesture_candidate.json
```

The output is intentionally non-commandable. Before a live test, a calibrated playback node must read the actual Dex3 hand state, establish a neutral position, confirm each motor's direction one at a time, then translate normalized values to clamped `HandCmd.motor_cmd[i].q` values. The local Unitree interface uses `rt/dex3/left/cmd` and `rt/dex3/right/cmd`; do not publish to those topics until that calibration is complete.

The supplied MuJoCo G1 model (`EF/ef_ws/g1/sim/walk_sim/unitree_robots/g1/g1_29dof.xml`) has fixed rubber-hand meshes, not articulated Dex3 finger joints. It can show the robot body and wrists, but cannot render this finger motion. A hand-articulated Dex3 URDF/MJCF is required for a visual replay.

## Real-hardware workflow

The two scripts below publish only to the Dex3 hand topics; they never move
the G1 arms, legs, or torso. They must be run beside a stationary robot with a
clear hand workspace and an accessible emergency stop.

First create a calibration. The default invocation is a no-motion safety
check; `--arm` is required before it can jog a motor. It captures the current
open/neutral pose and asks the operator to verify the flex direction of each
motor with a maximum 0.08 rad one-motor-at-a-time jog.

```bash
cd /home/unitree/motion-capture
python3 calibrate_dex3_hands.py
python3 calibrate_dex3_hands.py --arm --iface eth0 --domain-id 0
```

This creates `dex3_motion_calibration.json`. It stores only the visually
confirmed small flex deltas, not full joint limits.

Then inspect the capture-to-joint targets without moving the robot:

```bash
python3 play_dex3_motion.py
```

After reviewing that output and returning both hands to the calibrated neutral
pose, run at normal-or-slower speed with explicit arming:

```bash
python3 play_dex3_motion.py --arm --speed 0.5 --iface eth0 --domain-id 0
```

Playback refuses to start if live `HandState` feedback is missing or either
hand is too far from its saved neutral pose. It releases hand gains when it
finishes, loses feedback, or is interrupted.
