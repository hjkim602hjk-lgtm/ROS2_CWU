-- Cartographer 2D SLAM 설정입니다. slam.yaml의 Cartographer 버전이라고 보면 됩니다.
-- /scan(G4)과 /odom(엔코더)을 받아 map -> odom TF를 발행하며,
-- 점유 격자(/map)는 cartographer_occupancy_grid_node가 따로 만듭니다.
--
-- 프레임 구성: 엔코더(cwu_base)가 odom -> base_link를 이미 발행하므로
-- Cartographer는 그 위에 map -> odom만 얹습니다(provide_odom_frame = false).

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- IMU가 없으므로 추적 프레임은 base_link입니다. IMU를 달면 imu_link로 바꾸고
  -- 아래 use_imu_data를 true로 올려야 합니다.
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- 파이 4는 코어가 4개뿐이고 G4 드라이버·SLAM·나중의 카메라가 같이 돕니다.
-- 4로 두면 Cartographer가 코어를 전부 먹어 스캔 수신이 밀립니다.
MAP_BUILDER.num_background_threads = 2

-- G4 사양에 맞춘 거리 범위입니다. ydlidar_g4.yaml의 range_min/range_max와 같게 두십시오.
-- 드라이버는 무효 측정을 0.0으로 주는데, min_range 아래라 여기서 자동으로 버려집니다.
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- 지도가 흔들리면 아래 두 값을 키우고(오도메트리를 더 신뢰), 지도가 어긋나면 줄입니다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

-- 루프 클로저: 값이 낮을수록 자주 닫히지만 잘못 닫힐 위험도 커집니다.
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
