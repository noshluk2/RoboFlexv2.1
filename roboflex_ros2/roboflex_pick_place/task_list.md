# RoboFlex Pick and Place Task List

## Package Bootstrap

- [x] Create new ROS 2 package `roboflex_pick_place`
- [x] Add package metadata and install rules
- [x] Add initial package README

## External Camera + TF

- [x] Add launch for external Intel RealSense D455 integration
- [x] Add configurable static transform (camera -> robot base)
- [x] Add camera/scene config YAML

## Perception to MoveIt Scene

- [x] Add node to read RGB-D point cloud and filter workspace points
- [x] Add transform handling from camera frame to robot base frame
- [x] Add MoveIt PlanningScene collision-object publishing from segmented points

## Next Implementation Steps

- [ ] Multi-object segmentation (cluster extraction in point cloud)
- [ ] Object identity / pose estimation from RGB + depth
- [ ] Grasp candidate generation and ranking
- [ ] Pick and place task executor (MoveIt planning + execution)
- [ ] Calibration workflow docs for external camera extrinsics
- [ ] Full end-to-end demo launch and operator docs

## MVP Process (No ML First)

- [ ] Calibrate static transform `base_link -> camera_link` until point cloud and robot align in RViz
- [ ] Crop workspace in `base_link` and remove table plane from point cloud
- [ ] Cluster remaining points and publish object centroid + bounding box candidates
- [ ] Generate top-down grasp candidates (with pre-grasp and retreat offsets)
- [ ] Validate grasp candidates with MoveIt IK + collision checks
- [ ] Execute pick sequence: pre-grasp -> grasp -> close -> lift
- [ ] Execute place sequence: pre-place -> place -> open -> retreat
- [ ] Add retry logic (next candidate when plan/grasp fails)

## Data Recording Readiness Checklist

- [x] In RViz (`Fixed Frame = base_link`), table and objects are clearly visible in `/camera/depth/color/points`
- [x] Robot URDF in RViz and real robot overlap in the same frame (no obvious offset/drift)
- [ ] Camera distance/angle sees full pick workspace and gripper approach zone
- [x] Static TF (`camera_x/y/z`, `camera_roll/pitch/yaw`) is physically reasonable from tape-measure estimate
- [ ] Sanity test at 2-3 known points: EE tip vs cloud location error is small and consistent
- [ ] Segmented object clusters are stable for a static scene (no heavy flicker/jumping)
- [x] Required topics are present and stable before recording (`/tf`, `/tf_static`, `/joint_states`, point cloud, planning scene)
- [x] 30-60s dry-run rosbag recording + playback confirms data quality

## Captured Datasets (2026-02-23)

- [x] `arm_single_object_20260223_094215`
- [x] `moving_single_object_20260223_094411`
- [x] `multi_object_20260223_094553`
- [x] `wall_object_20260223_095036`

## Next Priority Tasks

- [ ] Implement `object_segmentation_node` (plane removal + multi-cluster extraction)
- [ ] Publish object hypotheses (`PoseArray` or custom message) in `base_link`
- [ ] Implement `pick_place_executor_node` state machine (pre-grasp -> grasp -> lift -> place -> retreat)
- [ ] Add MoveIt obstacle object insertion from segmented clusters (not only one box)
- [ ] Add runtime success/failure logging topic for each trial (`scene_id`, `plan_success`, `pick_success`)
- [ ] Run obstacle benchmark scenes (single wall, narrow gap, blocked path) and record pass/fail
- [ ] Tighten TF with 2-3 physical point checks (target <= 10 mm in pick zone)

## Stabilization

- [x] Fix runtime crash in `scene_integration_node` logger calls
- [x] Align external camera TF defaults to D455 launch frames (`camera_link`)
- [x] Add run instructions for `realsense2_camera` point cloud launch
- [x] Add URDF + RViz + static TF calibration launch
