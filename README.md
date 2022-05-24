## Install application

Download repository and checkout binary_task_planner branch:

`git clone https://github.com/RobotnikAutomation/hrrecycler_component_sorting -b ros_dev_day --recurse-submodules` 

Install binaries:

`cd src/hrrecycler_component_sorting/debs`

`sudo dpkg -i ros-melodic-*`

Compile repository:

`catkin build`
## Run application

To launch simulation + perception + navigation + localization:

`roslaunch component_sorting_bringup component_sorting_bringup.launch`

To launch MoveIt node (simulation must be running):

`roslaunch component_sorting_moveit_config component_sorting_moveit_config.launch`

Upload MoveIt custom constraints to the database (This step should only be carried out once):

`rosrun component_sorting_moveit_config  generate_path_constraints`

To launch manipulation application

`roslaunch component_sorting_manipulation manipulation_app.launch`


## Use application

How to use the manipulation application:

1. Use /docker action to perform docking to table:
    dock_frame: 'table_docking'
    robot_dock_frame: 'robot_base_docking_contact'

2. Launch manipulation application (simulation + MoveIt node must be running + robot positioned using docker):

    `roslaunch component_sorting_manipulation manipulation_app.launch`

3. Available manipulation actions: 
    - /component_sorting_manipulation_app/pick_object
    - /component_sorting_manipulation_app/place_object

4. Available Scene Manager services: 
    - /scene_manager/add_objects
    - /scene_manager/attach_objects
    - /scene_manager/detach_objects
    - /scene_manager/modify_object
    - /scene_manager/move_relative_to
    - /scene_manager/remove_objects


