## run

To launch simulation + perception + navigation + localization:

`roslaunch component_sorting_bringup component_sorting_bringup.launch`

To launch MoveIt Database:

`roslaunch component_sorting_moveit_config default_warehouse_db.launch`

To launch MoveIt (simulation + database must be running):

`roslaunch component_sorting_moveit_config component_sorting_moveit_config.launch`

To launch MoveIt RVIZ: 

`roslaunch component_sorting_moveit_config moveit_rviz.launch`

To launch component sorting MoveIt application (MoveIt RVIZ must be running):

`roslaunch component_sorting component_sorting_cartesian.launch`

To launch boxes into simulation: 
 - In table : `roslaunch component_sorting_simple_sim boxes_table.launch`
 - In kairos: `roslaunch component_sorting_simple_sim boxes.launch`

Component sorting actions:

- `rosrun actionlib axclient.py /component_sorting_cartesian/pickup_from`

- `rosrun actionlib axclient.py /component_sorting_cartesian/place_on`
