## run

To launch simulation + perception + navigation + localization:

`roslaunch component_sorting_bringup component_sorting_bringup.launch`

To launch boxes on top of table:

`roslaunch component_sorting_simple_sim boxes_table.launch`

To launch MoveIt node (simulation must be running):

`roslaunch component_sorting_moveit_config component_sorting_moveit_config.launch`

Upload MoveIt custom constraints to the database (This step should only be carried out once):

`rosrun component_sorting_moveit_config  generate_path_constraints`


To launch MoveIt RVIZ (once move_group node is ready): 

`roslaunch component_sorting_moveit_config moveit_rviz.launch`


How to launch and use the component_sorting manipulation application:
1. Navigate towards table until table qr is visible from kairos frontal camera (/front_rgbd_camera/rgb/image topic)

2. Use /docker action to perform docking to table:
    dock_frame: 'table_docking'
    robot_dock_frame: 'robot_base_docking_contact'

3. Launch component sorting MoveIt manipulation application (simulation + MoveIt node must be running + robot positioned):

`roslaunch component_sorting component_sorting.launch`


Component sorting actions:

Init holder action -> used to scan holder qr position of the desired available position* (if left empty it will scan all available positions)

- `rosrun actionlib axclient.py /component_sorting/init_holder` 

Pickup from action -> used to pick a box from the desired available position* (pick position is computed in relation with the box qr position which can be visualized by the arm camera from the pre-pick position of the desired position)

- `rosrun actionlib axclient.py /component_sorting/pickup_from` 

Place on action -> used to place a box in the desired available position* (desired holder position has to be initialized before trying to place box as 
place position is computed in relation to the desired holder qr position)

- `rosrun actionlib axclient.py /component_sorting/place_on`

*The available positions can be found in positions_to_use parameter inside /component_sorting/config/poses_config.yaml)

    positions_to_use:
    - robot_center
    - robot_right
    - robot_left
    - table_center
    - table_right
    - table_left