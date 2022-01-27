## Install application

Download repository and checkout binary_task_planner branch:

`git clone https://github.com/RobotnikAutomation/hrrecycler_component_sorting -b binary_task_planner --recurse-submodules` 

Install binaries:

`cd src/hrrecycler_component_sorting\debs`
`sudo dpkg -i ros-melodic-*`

Compile repository:

`catkin build`
## Run application

To launch simulation + perception + navigation + localization:

`roslaunch component_sorting_bringup component_sorting_bringup.launch`

To launch box on top of table:

`roslaunch component_sorting_simple_sim box_table.launch`

To launch MoveIt node (simulation must be running):

`roslaunch component_sorting_moveit_config component_sorting_moveit_config.launch`

Upload MoveIt custom constraints to the database (This step should only be carried out once):

`rosrun component_sorting_moveit_config  generate_path_constraints`


To launch MoveIt RVIZ (once move_group node is ready): 

`roslaunch component_sorting_moveit_config moveit_rviz.launch`

Component sorting actions:

Move to action -> Used to move the end-effector to a predefined cartesian position from which the arm camera can detect the box in the table. This action can be used send end-effector to one of the predefined spots* or to positions previously stored in srdf. 

- `rosrun actionlib axclient.py /component_sorting/move_to` 

Move to pose action -> Used to move the end-effector to a cartesian position from which the arm camera can detect the box in the table. The goal pose is defined using a geometry_msgs/PoseStamped msg. (Can be used instead of move_to action if prefered)

- `rosrun actionlib axclient.py /component_sorting/move_to_pose` 

Pickup from action -> used to lift a box from any given spot* (either the table or one of the spots in the Kairos's back), once the action is completed the RB-Kairos's gripper will have the box attached by the handle. 

- `rosrun actionlib axclient.py /component_sorting/pickup_from` 

Place on action -> used to carefully place the box that is attached to the arm into the desired spot.* 

- `rosrun actionlib axclient.py /component_sorting/place_on`

*The available spots can be found and configured inside /component_sorting/config/poses_config.yaml)

    positions_to_use:
    - robot_center
    - robot_right
    - robot_left
    - table

## Use application

How to launch and use the component_sorting manipulation application:

1. Use /docker action to perform docking to table:
    dock_frame: 'table_docking'
    robot_dock_frame: 'robot_base_docking_contact'

2. Launch component sorting MoveIt manipulation application (simulation + MoveIt node must be running + robot positioned):

`roslaunch component_sorting component_sorting.launch`

3. Use move_to "table" action to move end effector to table approach position (table approach pose is already defined in poses_config.yaml for each available position). From this pose the box qr can be seen by the arm camera.

4. Use pickup_from "table" action to pick box from table position. Pick and pre-pick poses are already defined for each position in poses_config.yaml.

5. Use place_on "robot_..." action to place box in desired robot position (right, left, center). Place and pre-place poses are already defined for each position in poses_config.yaml.

6. To pick boxes from robot positions the detection algorithm is not needed, pickup_from action can be directly used (without moving to an approach position to see the qr or detect the box).