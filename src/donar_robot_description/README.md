# Donar Single-Robot Final Project Workflow

This package already has the pieces you need for the single-robot project flow:

- saved-map localization with `nav2_map_server` + `AMCL`
- predefined multi-goal navigation through `mission_planner.py`
- an RViz goal-capture helper
- a Gazebo dynamic obstacle for replanning tests

For the single-robot demo path, use the saved map and AMCL for localization. That is the clean production-style flow for this workspace after map creation. You only need `slam_toolbox` during the mapping stage.

## 1. Build And Source

```bash
cd /home/ganesh/robot_ws
colcon build --packages-select donar_robot_description
source /opt/ros/humble/setup.bash
source /home/ganesh/robot_ws/install/setup.bash
```

## 2. Save The Map Once

If you are still creating the map live, save it with:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/ganesh/robot_ws/warehouse_map
```

This workspace already contains:

- `/home/ganesh/robot_ws/warehouse_map.yaml`
- `/home/ganesh/robot_ws/warehouse_map.pgm`

## 3. Localize On The Saved Map

Use the single-robot Nav2 launch to run Gazebo, load the saved map, start AMCL, and publish the initial pose:

```bash
ros2 launch donar_robot_description single_robot_nav2.launch.py \
  map:=/home/ganesh/robot_ws/warehouse_map.yaml \
  use_rviz:=true
```

Notes:

- `single_robot_nav2.launch.py` already uses `nav2_bringup` with `slam:=False`, so it loads the saved map instead of remapping every run.
- The default spawn and initial pose are both `x=-1.0`, `y=-3.0`, `yaw=0.0`.

## 4. Capture A Predefined Route

Use RViz `2D Goal Pose` clicks to create your shelf route file:

```bash
ros2 run donar_robot_description capture_goal_poses.py --ros-args \
  -p output_file:=/home/ganesh/robot_ws/src/donar_robot_description/config/warehouse_route.json
```

Then in RViz:

1. Click `2D Goal Pose` for each shelf or drop-off point in order.
2. Stop the node when the route is complete.
3. Reuse that JSON file for repeated demos.

The package also includes an example route file:

- [/home/ganesh/robot_ws/src/donar_robot_description/config/final_demo_goals.json](/home/ganesh/robot_ws/src/donar_robot_description/config/final_demo_goals.json)

## 5. Run Predefined Multi-Goal Navigation

For the route only:

```bash
ros2 launch donar_robot_description predefined_multi_goal.launch.py \
  map:=/home/ganesh/robot_ws/warehouse_map.yaml \
  goals_file:=/home/ganesh/robot_ws/src/donar_robot_description/config/warehouse_route.json \
  use_rviz:=true
```

For the full project-style demo with the mission planner and optional obstacle:

```bash
ros2 launch donar_robot_description final_project_demo.launch.py \
  map:=/home/ganesh/robot_ws/warehouse_map.yaml \
  goals_file:=/home/ganesh/robot_ws/src/donar_robot_description/config/warehouse_route.json \
  use_rviz:=true \
  enable_dynamic_obstacle:=false
```

Useful monitoring commands:

```bash
ros2 topic echo /mission_status
ros2 topic echo /amcl_pose --once
ros2 topic list | grep mission
```

`mission_planner.py` is the Step 4 custom node. It already:

- loads a queue of goal poses from JSON
- sends them to Nav2 one by one
- retries failed goals
- publishes status on `/mission_status`
- publishes RViz markers on `/mission_goals`

## 6. Validate Dynamic Obstacle Replanning

Enable the obstacle in the full demo launch:

```bash
ros2 launch donar_robot_description final_project_demo.launch.py \
  map:=/home/ganesh/robot_ws/warehouse_map.yaml \
  goals_file:=/home/ganesh/robot_ws/src/donar_robot_description/config/warehouse_route.json \
  use_rviz:=true \
  enable_dynamic_obstacle:=true
```

What to verify:

- the red box appears in Gazebo
- the local costmap updates as it crosses the robot path
- the robot slows, deviates, or replans instead of colliding
- `/mission_status` still progresses toward `COMPLETED`

## 7. What To Record For Evaluation

- screenshot of RViz map versus the Gazebo warehouse world
- screen recording of one full multi-goal run
- notes on how many goals succeeded on the first try
- any `RETRYING` states from `/mission_status`
- at least one visible replanning event when the obstacle crosses the path

## 8. Suggested Cleanup Candidates

I did not delete these yet because they are project-history files and removing them is destructive:

- `launch/multi_robot_gazebo.launch.py`
- `launch/multi_robot_nav2.launch.py`
- `launch/multi_robot_full_stack.launch.py`
- `launch/multi_robot_slam.launch.py`
- `config/nav2_params_multi_robot.yaml`
- `config/slam_params_multi_robot.yaml`

If you fully want to pivot this repository to single-robot only, those are the first files I would remove after you confirm you do not need multi-robot recovery later.