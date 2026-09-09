# Encoder-assisted 2D SLAM

User confirmed wheel encoders after the proposed Humble/G4/SLAM Toolbox design.
Target: Ubuntu 22.04, ROS 2 Humble, Raspberry Pi 4 4GB. All mission computation
runs onboard; no prebuilt arena map or external communication is required.

Create `cwu_slam`, an ament_python bringup package. SLAM Toolbox consumes `/scan`
and the encoder driver's timestamped `odom -> base_link` TF. The encoder driver
should also publish `nav_msgs/Odometry` on `/odom`; SLAM Toolbox uses TF, not that
topic. Do not invent wheel geometry, tick encoding, robot kinematics, or transport.
The driver implementation is outside this package until those are specified.

The package owns `map -> odom` via SLAM Toolbox and optionally a static
`base_link -> laser_frame` mount transform. Real bringup starts the G4 driver;
an existing URDF can instead own the mount transform. Hardware calibration is
required before deployment, never arena-specific post-reveal tuning.

A separate demo launch supplies synthetic scans and exact odometry on a closed
trajectory in an asymmetric room. It uses wall time, not a nonexistent /clock.
The synthetic map is only a test fixture and never loaded in real mode.
The demo has no motor command publisher. Parameters live in YAML. RViz is optional
and off by default. Map saving is explicit through Nav2 map_saver_cli.

Validation: ray/segment numerical unit tests; colcon build/test; launch the real
SLAM backend against synthetic inputs, verify free and occupied map cells and
the TF chain, and save/reload a map artifact. This proves integration, not encoder
accuracy, G4 hardware operation, loop closure quality, or Raspberry Pi throughput.
Missing competition_rules.md means rules are taken only from Agent.md.
