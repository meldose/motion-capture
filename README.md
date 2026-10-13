# Motion Studio to G1 Dex3: offline preparation

`export_dex3_candidate.py` turns the cached Motion Studio hand-bone quaternions into a 20 Hz normalized gesture recording. It does **not** connect to ROS/DDS or command the robot. The source capture is timed at about 120.7 Hz (3,259 frames in ~27 seconds), and the converter preserves that timing.

The recording contains these source bones: thumb, index, and middle finger for both hands. Dex3 has seven motors per hand in this order:

`thumb_0, thumb_1, thumb_2, index_0, index_1, middle_0, middle_1`

Run from this directory:

```bash
python3 export_dex3_candidate.py ../.ms/cache/algo/6kl2x8slyswzguvw3njghlpj52tcqtlk_12691235622399232648 dex3_gesture_candidate.json
```

The output is intentionally non-commandable. Before a live test, a calibrated playback node must read the actual Dex3 hand state, establish a neutral position, confirm each motor's direction one at a time, then translate normalized values to clamped `HandCmd.motor_cmd[i].q` values. The local Unitree example uses `/dex3/left/cmd` and `/dex3/right/cmd`; do not publish to those topics until that calibration is complete.
