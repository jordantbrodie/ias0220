# Task 5.1 submission, student 243483

This ZIP contains only Task 5.1, with a task-specific CMakeLists.txt and package.xml.
The full coursework package and previous recordings remain unchanged outside the ZIP.
Install ROS 2 Jazzy and the instructor ias0220_sensors package plus the dependencies
listed in package.xml. Extract this package into a clean workspace src directory,
source /opt/ros/jazzy/setup.bash, run colcon build --packages-select ias0220_243483,
and source install/setup.bash.

From the extracted package directory:

```bash
ros2 launch ias0220_243483 bagfile_play_RViz_task5_part1.launch.py bag_path:=bags/recorded_take6
```

Recording defaults to bags/recorded and refuses an existing output directory.
No sensor is needed for playback. Earlier takes are deliberately excluded.
The assignment requests a bag named recorded; Take 6 retains its identifying name
and is selected explicitly with bag_path, without duplicating or renaming raw data.

Evidence: screenshot 1 is supplied as two final Take 6 images:
data/take6_orientation.png and data/take6_distance.png. Explain this split in the
Moodle submission comment; these replace plotjuggler_offline_bag.png.
Orientation x/y/z are quaternion components, not Euler angles in degrees.
The online screenshot contains both rqt_plot and PlotJuggler.
The RViz screenshot shows axes and Global Status OK, but the range cone is not
clearly visible. A replacement showing both axes and cone is recommended before upload.

Verification on 2026-10-22:
- Main coursework package colcon build passed; both launch files parse and ROS lists their arguments.
- Take 6: 56.853111125 s; 1455 messages; /imu 479, /distance 479,
  /tf 479, /tf_static 1, /rosout 17. All messages read and deserialized.
- Distance range includes negative sensor returns: approximately -0.01 to 3.83 m.
  Raw recording is preserved unchanged.
- Recording launch inspected against instructor serial_port parameter and frame names.
- Playback supplies --clock; RViz uses use_sim_time=true; map is the fixed frame.
- Playback process started, but sandbox socket restrictions and unavailable X display
  prevented end-to-end ROS communication and RViz rendering verification.
- Physical recording was reported completed by the student; no new recording was made.
- No video is required by the task PDF. Feedback is requested for bonus points.

Before Moodle upload, check/re-capture RViz evidence with a visible range cone,
and replay Take 6 on the normal desktop. The original screenshot is preserved.
