#include <component_sorting/component_sorting.h>

ComponentSorting::ComponentSorting(ros::NodeHandle h) : RComponent(h)
{
  init(h);
}
ComponentSorting::~ComponentSorting()
{
}

void ComponentSorting::rosReadParams()
{
  bool required = true;

  required = true;
  group_name_ = "arm";
  readParam(pnh_, "group_name", group_name_, group_name_, required);

  required = true;
  host = "localhost";
  readParam(pnh_, "host", host, host, required);  

  required = true;
  port = 33829;
  readParam(pnh_, "port", port, port, required);

  required = false;
  double timeout = 20;
  readParam(pnh_, "move_group_timeout", timeout, timeout, required);
  move_group_timeout_ = ros::WallDuration(timeout);

  required = true;
  box.width = 0.20;
  readParam(pnh_, "box/width", box.width, box.width, required);

  required = true;
  box.length = 0.28;
  readParam(pnh_, "box/length", box.length, box.length, required);

  required = true;
  box_handle_displacement = 0.05;
  readParam(pnh_, "box_handle_displacement", box_handle_displacement, box_handle_displacement, required);
  
  required = true;
  moveit_constraint = "";
  readParam(pnh_, "moveit_constraint", moveit_constraint, moveit_constraint, required); 

}

int ComponentSorting::rosSetup()
{
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }

  tf2_buffer_.reset(new tf2_ros::Buffer);

  try
  {
    move_group_.reset(
        new moveit::planning_interface::MoveGroupInterface(group_name_, tf2_buffer_, move_group_timeout_));
  }
  catch (const std::runtime_error& e)
  {
    RCOMPONENT_ERROR("Cannot create move group with group name: %s. Is MoveIt running? Group name is correct?",
                     group_name_.c_str());
    RCOMPONENT_ERROR_STREAM("Exception: " << e.what());
    return rcomponent::ERROR;
  }

  bool autostart = false;
  pickup_from_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>(pnh_, "pickup_from", autostart));
  pickup_from_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("pickup_from")));
  pickup_from_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  place_on_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>(pnh_, "place_on", autostart));
  place_on_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("place_on")));
  place_on_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  init_holder_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::InitHolderAction>(pnh_, "init_holder", autostart));
  init_holder_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("init_holder")));
  init_holder_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));

  conn_ = moveit_warehouse::loadDatabase();
  conn_->setParams(host, port);

  ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);

  while (!conn_->connect())
  {
    ROS_ERROR("Failed to connect to DB on %s:%d ", host.c_str(), port);
    ros::Duration(2).sleep();
    conn_->setParams(host, port);
  }
  

  move_group_->setConstraintsDatabase(host,port);
  std::vector< std::string > stored_constraints = move_group_->getKnownConstraints();
  if (stored_constraints.empty())
    ROS_WARN("There are no constraints stored in database");
  else
  {
    ROS_INFO("Constraints currently stored in database:");
    for (const std::string& name : stored_constraints)
      ROS_INFO(" * %s", name.c_str());
  }


  // Read and store parameter: approach_poses from parameter server
  bool required = true;
  readParam(pnh_, "positions_to_use", positions_to_use, positions_to_use, required); 

  ros::NodeHandle pnh_approach_ = ros::NodeHandle(pnh_ , "approach_poses");
  ros::NodeHandle pnh_place_ = ros::NodeHandle(pnh_ , "place_poses");
  ros::NodeHandle pnh_pick_ = ros::NodeHandle(pnh_ , "pick_poses");
  ros::NodeHandle pnh_pre_pick_ = ros::NodeHandle(pnh_ , "pre_pick_poses");
  ros::NodeHandle pnh_pre_place_ = ros::NodeHandle(pnh_ , "pre_place_poses");
/*   readParam(pnh_approach_, "poses_to_use", approach_poses_names, approach_poses_names, required);  */

  // Create class Pose Builder objects
  for (auto & position: positions_to_use)
  {
    if(pnh_approach_.hasParam(position)){
      approach_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_approach_ , position))));
    }else{
      ROS_ERROR("Approach pose has to be filled in for %s position", position.c_str());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return -1;
    }

    if(pnh_place_.hasParam(position)){
      place_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_place_ , position))));
    }else{
      ROS_ERROR("Place pose has to be filled in for %s position", position.c_str());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return -1;
    }

    if(pnh_pick_.hasParam(position)){
      pick_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_pick_ , position))));
    }else{
      ROS_ERROR("Pick pose has to be filled in for %s position", position.c_str());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return -1;
    }    

    if(pnh_pre_pick_.hasParam(position)){
      pre_pick_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_pre_pick_ , position)))); 
    }else{
      ROS_ERROR("Pre_pick pose has to be filled in for %s position", position.c_str());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return -1;
    }    
    
    if(pnh_pre_place_.hasParam(position)){
      pre_place_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_pre_place_ , position))));
    }else{
      ROS_ERROR("Pre_place pose has to be filled in for %s position", position.c_str());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return -1;
    }  

  }


  // Gazebo link_attacher service
  client = nh_.serviceClient<ur_msgs::SetIO>("arm/ur_hardware_interface/set_io");
  gazebo_link_attacher_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  gazebo_link_detacher_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");



  //UNCOMMENT FOR VISUALIZATION
/*   visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("robot_base_footprint","/move_group/display_grasp_markers"));
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger(); */


  // in case we contact MoveIt through actionlib
  // pickup_as_.reset(new actionlib::SimpleActionServer<moveit_msgs::PickupAction>(pnh_, "pickup", autostart));
  // pickup_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this));
  // pickup_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  // place_as_.reset(new actionlib::SimpleActionServer<moveit_msgs::PlaceAction>(pnh_, "place", autostart));
  // place_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this));
  // place_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  //
  // bool spin_action_thread = true;
  // pickup_ac_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(nh_, "pickup", spin_action_thread));
  // place_ac_.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(nh_, "place", spin_action_thread));

  // TF Listener and Broadcaster
  tf_latch_timer = pnh_.createTimer(ros::Duration(0.1), std::bind(&ComponentSorting::tfLatchCallback, this));
  tf_listener = new  tf2_ros::TransformListener(tfBuffer);

  return RComponent::rosSetup();
}

int ComponentSorting::rosShutdown()
{
  return RComponent::rosShutdown();
}

int ComponentSorting::setup()
{
  // Checks if has been initialized
  int setup_result;

  setup_result = rcomponent::RComponent::setup();
  if (setup_result != rcomponent::OK)
  {
    return setup_result;
  }

  // in case we contact MoveIt through actionlib
  // ros::Duration timeout = ros::Duration(5);
  // pickup_ac_->waitForServer(timeout);
  // if (pickup_ac_->isServerConnected() == false)
  // {
  //   RCOMPONENT_ERROR_STREAM("Cannot connect to pick up client: pickup");
  //   return rcomponent::ERROR;
  // }
  // place_ac_->waitForServer(timeout);
  // if (place_ac_->isServerConnected() == false)
  // {
  //   RCOMPONENT_ERROR_STREAM("Cannot connect to pick up client: place");
  //   return rcomponent::ERROR;
  // }

  // pickup_as_->start();
  // RCOMPONENT_INFO_STREAM("Started server: pickup");
  // place_as_->start();
  // RCOMPONENT_INFO_STREAM("Started server: place");

  pickup_from_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: pickup from");
  place_on_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: place on");
  init_holder_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: init holder");

  // Create planning scene
  if(create_planning_scene()){
    return rcomponent::OK;
  }else{
    return rcomponent::ERROR;
  }
}

void ComponentSorting::standbyState()
{

//UNCOMMENT FOR VISUALIZATION
/*   robot_state_ = move_group_->getCurrentState();
  joint_model_group= robot_state_->getJointModelGroup("arm"); */

  // Move to home position without selected constraints
  move_group_->detachObject();  
  move_group_->clearPathConstraints();
  move_group_->setNamedTarget("home_position");
  success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success_move){

    ROS_INFO("Moved to home position, ready to take commands");

    // Select constraint
    move_group_->setPathConstraints(moveit_constraint);

    current_constraint = move_group_->getPathConstraints();

    if(moveit_constraint != current_constraint.name){
      ROS_ERROR("Desired moveit_constraint is not available in database, please modify or run generate_path_constraints.cpp");
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return;
    }

    switchToState(robotnik_msgs::State::READY_STATE);

  }else{
    ROS_WARN("Could not move to home position");
  }

}

void ComponentSorting::readyState()
{ 

  if (pickup_from_as_->isActive() == false && place_on_as_->isActive() == false && init_holder_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }

  ROS_INFO_THROTTLE(3, "I have a new goal!");

  // Select constraint
  move_group_->setPathConstraints(moveit_constraint);
  //move_group_->clearPathConstraints (); //Remove
  current_constraint = move_group_->getPathConstraints();

  if(moveit_constraint != current_constraint.name){
    if(pickup_from_as_->isActive() == true){ 
      pick_result_.success = false;
      pick_result_.message = "Moveit constraint not available in database";
      pickup_from_as_->setAborted(pick_result_);
    }

    if(place_on_as_->isActive() == true){
      place_result_.success = false;
      place_result_.message = "Moveit constraint not available in database";
      place_on_as_->setAborted(place_result_);
    }
  
  }

  ROS_INFO("Planning with Constraint: %s", current_constraint.name.c_str());


  if(init_holder_as_->isActive() == true){ 

    // Get string of positions
    std::vector <std::string> scanning_positions = init_holder_goal_->position; 

    if(scanning_positions.empty()){
      for (auto & position: positions_to_use)
      {
        scan(position);
      }
    }else{
      for (auto & scanning_position: scanning_positions){
        if(find(positions_to_use.begin(), positions_to_use.end(),scanning_position) != end(positions_to_use)){
          scan(scanning_position);
        }else {
          ROS_WARN("Position %s does not exist", scanning_position.c_str());
          init_holder_feedback_.state.clear();
          init_holder_feedback_.state = scanning_position + " position does not exist";
          init_holder_as_->publishFeedback(init_holder_feedback_);
        }
      }
    }

    init_holder_result_.success = true;
    init_holder_result_.message = "Vector of positions processed ";
    init_holder_as_->setSucceeded(init_holder_result_);
    return;
  }

  if(pickup_from_as_->isActive() == true){ 

    // Get desired goal and set as target
    std::string pick_position = pickup_from_goal_->from;
    //Check if position exists
    if(find(positions_to_use.begin(), positions_to_use.end(),pick_position) != end(positions_to_use)){
      pick_chain_movement(pick_position);
    }else {
      ROS_WARN("Position %s does not exist", pick_position.c_str());
            
      pick_result_.success = false;
      pick_result_.message = "Position does not exist";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }
    
  }   
  
  if(place_on_as_->isActive() == true){ 
    // Get desired goal and set as target
    std::string place_position = place_on_goal_->in;

    //Check if position exists
    if(find(positions_to_use.begin(), positions_to_use.end(),place_position) != end(positions_to_use)){
      //Check if place position is initialized
      if( place_poses_.at(place_position).isInit()){
        place_chain_movement(place_position);
      }else{
        ROS_WARN("Place pose for Position %s is not initialized ", place_position.c_str());

        place_result_.success = false;
        place_result_.message = "Place pose not initialized";
        place_on_as_->setAborted(place_result_);
        return;
      }
    }else {
      ROS_WARN("Position %s does not exist", place_position.c_str());

      place_result_.success = false;
      place_result_.message = "Position does not exist";
      place_on_as_->setAborted(place_result_);
      return;
    }
    
  }   
}

void ComponentSorting::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  action_ = action;
  if (action_ == "pickup_from"){
    pickup_from_goal_ = pickup_from_as_->acceptNewGoal();}
  if (action_ == "place_on"){
    place_on_goal_ = place_on_as_->acceptNewGoal();}
  if (action_ == "init_holder"){
    init_holder_goal_ = init_holder_as_->acceptNewGoal();}  
    
}

void ComponentSorting::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  //result_.success = false;
  //result_.message = "Goal has been cancelled, stopping execution.";
  // set the action state to preempted
  pickup_from_as_->setPreempted();
  place_on_as_->setPreempted();
  init_holder_as_->setPreempted();
}

void ComponentSorting::tfListener(std::string scanning_position){
/*   tf_listener.lookupTransform("robot_base_link",frame_name,ros::Time(0),transform); */
  std::string frame_name = place_poses_.at(scanning_position).getPose().header.frame_id;
  try{
    transform_stamped = tfBuffer.lookupTransform("robot_base_link",frame_name,ros::Time(0));
  }
  catch(tf2::LookupException ex){
    ROS_WARN("Lookup Transform error: %s", ex.what());
    if(init_holder_as_->isActive() == true){ 
      init_holder_feedback_.state.clear();
      init_holder_feedback_.state = "Could not scan " + scanning_position;
      init_holder_as_->publishFeedback(init_holder_feedback_);
    }
    return;
  }
  catch(tf2::ExtrapolationException ex){
    ROS_WARN("Lookup Transform error: %s", ex.what());
    if(init_holder_as_->isActive() == true){ 
      init_holder_feedback_.state.clear();
      init_holder_feedback_.state = "Could not scan " + scanning_position;
      init_holder_as_->publishFeedback(init_holder_feedback_);
    }
    return;
  }
  //Check if position is already published and remove it from vector
  for(int i=0; i<latched_tf.size(); i++){
    if(latched_tf[i].child_frame_id == transform_stamped.child_frame_id){
      latched_tf.erase(latched_tf.begin() + i);
    }
  }

  //Latch current visualized frame
  latched_tf.push_back(transform_stamped);
  //Set position holder frame as initialized
  place_poses_.at(scanning_position).setInit();
  //If init_holder action is runnning
  if(init_holder_as_->isActive() == true){ 
    init_holder_feedback_.state.clear();
    init_holder_feedback_.state = scanning_position + " position was scanned succesfully";
    init_holder_as_->publishFeedback(init_holder_feedback_);
  }
}

void ComponentSorting::tfLatchCallback(){
  for(auto tf : latched_tf){
    std::string name = tf.child_frame_id + "_latched";
/*     tf.child_frame_id_ = name;
    tf.stamp_ = ros::Time::now(); */
    tf.child_frame_id = name;
    tf.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(tf);
  }
}

void ComponentSorting::scan(std::string scanning_position)
{ 

  //Plan to approach 
  move_group_->setPoseTarget(approach_poses_.at(scanning_position).getPose());
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  //If plan is successful execute trajectory
  if(success_plan){
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success_execute){
      tfListener(scanning_position);
    }
  }
}

void ComponentSorting::pick_chain_movement(std::string pick_position)
{  
  //Extract position poses:
  geometry_msgs::PoseStamped approach_position = approach_poses_.at(pick_position).getPose();
  geometry_msgs::PoseStamped pre_position = pre_pick_poses_.at(pick_position).getPose();
  geometry_msgs::PoseStamped position = pick_poses_.at(pick_position).getPose();
 
/*   visual_tools_->deleteAllMarkers();
  visual_tools_->trigger(); */
  // Set pre-position goal
  move_group_->setPoseTarget(approach_position);
  // Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
 
/*   visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools_->trigger(); */
  //If plan is successful execute trajectory
  if(success_plan){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan to desired pre-position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to pre-position goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //ros::Duration(1).sleep();

    //Check if execute is successful
    if(success_execute){
      pick_feedback_.state.clear();
      pick_feedback_.state = "Moved to desired pre-position";
      pickup_from_as_->publishFeedback(pick_feedback_);
    }else{
      pick_result_.success = false;
      pick_result_.message = "Could not move to desired pre-position";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }
          
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not plan to desired pre-position";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  // Check if goal is active 
  //if (!pickup_from_as_->isActive()) return;
  // Get current end effector position and check whether there is a box to grab
  current_cartesian_pose = move_group_->getCurrentPose().pose;

  objects_in_roi.clear();
  objects_in_roi = planning_scene_interface.getKnownObjectNamesInROI(current_cartesian_pose.position.x - box.length/2, current_cartesian_pose.position.y - box.width/2 ,0, current_cartesian_pose.position.x 
  + box.length/2, current_cartesian_pose.position.y + box.width/2 , 3, false );


  if(objects_in_roi.size() < 2){
    pick_result_.success = false;
    pick_result_.message = "There is no box to grab";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  for(int i=0; i < objects_in_roi.size(); i++){
   if(objects_in_roi[i].compare(0,6,"handle")==0){
     identified_handle = objects_in_roi[i];
   }else if (objects_in_roi[i].compare(0,3,"box")==0) {
      identified_box = objects_in_roi[i];
   }else {}
  }

  // Move to pre-position
  move_group_->setPoseTarget(pre_position);
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success_plan)
    { 
      success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

  // Cartesian move to position  
  waypoints.clear();
  waypoint_cartesian_pose = position.pose;
  waypoints.push_back(waypoint_cartesian_pose); 

  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

/*   visual_tools_->publishTrajectoryLine(cartesian_plan.trajectory_, joint_model_group);
  visual_tools_->trigger(); */


  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan to desired position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::Duration(1).sleep();

    //Check if execute is successful
    if(success_execute){
      pick_feedback_.state.clear();
      pick_feedback_.state = "Moved to desired position";
      pickup_from_as_->publishFeedback(pick_feedback_);
    }else{
      pick_result_.success = false;
      pick_result_.message = "Could not move to desired position";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }       
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not plan to desired position";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  //Pick object identified as handle
  if(move_group_->attachObject(identified_handle)){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Handle attached to end effector";
    pickup_from_as_->publishFeedback(pick_feedback_);
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not attach handle";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  //Pick handle in gazebo
  gazebo_link_attacher_msg.request.model_name_1 = identified_box;
  gazebo_link_attacher_msg.request.link_name_1 = identified_box + "_handle_link";
  gazebo_link_attacher_msg.request.model_name_2 = "robot";
  gazebo_link_attacher_msg.request.link_name_2 = "robot_arm_wrist_3_link";

  if(gazebo_link_attacher_client.call(gazebo_link_attacher_msg)){
    /* std:: cout << "ok" << endl; */
  };

  //Activate gripper
  gripper_on();
  ros::Duration(1).sleep();

  //Move 5cm upwards and attach box
  waypoints.clear();
  waypoint_cartesian_pose = position.pose;
  waypoint_cartesian_pose.position.z += box_handle_displacement; 
  waypoints.push_back(waypoint_cartesian_pose);  

  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);


  if(move_group_->attachObject(identified_box)){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Box attached to end effector";
    pickup_from_as_->publishFeedback(pick_feedback_);
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not attach box";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  // Cartesian move to pre-position
  waypoints.clear();
  waypoint_cartesian_pose = pre_position.pose;
  waypoints.push_back(waypoint_cartesian_pose);  

  move_group_->setPoseReferenceFrame(pre_position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);

  // Move back to approach position
  waypoints.clear();
  waypoint_cartesian_pose = approach_position.pose;
  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(approach_position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

    
/*   visual_tools_->publishTrajectoryLine(cartesian_plan.trajectory_, joint_model_group);
  visual_tools_->trigger(); */

  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan back to pre-position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //ros::Duration(3).sleep();

    //Check if execute is successful
    if(success_execute){
      pick_feedback_.state.clear();
      pick_feedback_.state = "Moved back to pre-position";
      pickup_from_as_->publishFeedback(pick_feedback_);

      pick_result_.success = true;
      pick_result_.message = "Pick-up from desired position SUCCESSFUL";
      pickup_from_as_->setSucceeded(pick_result_);
      return;
    }else{
      pick_result_.success = false;
      pick_result_.message = "Could not move back to pre-position";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }          
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not plan back to pre-position";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

}

void ComponentSorting::place_chain_movement(std::string place_position)
{ 
  //EXtract place_position poses
  geometry_msgs::PoseStamped pre_position = pre_place_poses_.at(place_position).getPose();
  geometry_msgs::PoseStamped position = place_poses_.at(place_position).getPose();

  //Correct frame to latched 
  pre_position.header.frame_id = pre_position.header.frame_id + "_latched";
  position.header.frame_id = position.header.frame_id + "_latched";

/*   visual_tools_->deleteAllMarkers();
  visual_tools_->trigger(); */

  // Set pre-position goal
  move_group_->setPoseTarget(pre_position);
  //Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
/*   visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools_->trigger();  */ 
  //If plan is successful execute trajectory
  if(success_plan){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan to desired pre-position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::Duration(1).sleep();

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
      place_feedback_.state.clear();
      place_feedback_.state = "Moved to desired pre-position";
      place_on_as_->publishFeedback(place_feedback_);
    }else{
      place_result_.success = false;
      place_result_.message = "Could not move to desired pre-position";
      place_on_as_->setAborted(place_result_);
      return;
    }
          
  }else{
    place_result_.success = false;
    place_result_.message = "Could not plan to desired pre-position";
    place_on_as_->setAborted(place_result_);
    return;
  }

  //Plan to position goal + 0.05 m 

  waypoints.clear();
  waypoint_cartesian_pose = position.pose;
  waypoint_cartesian_pose.position.z += box_handle_displacement;
  waypoints.push_back(waypoint_cartesian_pose); 

  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;


/*   visual_tools_->publishTrajectoryLine(cartesian_plan.trajectory_, joint_model_group);
  visual_tools_->trigger(); */
  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan to desired position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::Duration(2).sleep();

    //Check if execute is successful
    if(success_execute){
      place_feedback_.state.clear();
      place_feedback_.state = "Moved to desired position";
      place_on_as_->publishFeedback(place_feedback_);
    }else{
      place_result_.success = false;
      place_result_.message = "Could not move to desired position";
      place_on_as_->setAborted(place_result_);
      return;
    }
          
  }else{
    place_result_.success = false;
    place_result_.message = "Could not plan to desired position";
    place_on_as_->setAborted(place_result_);
    return;
  }

  //Detach box from end effector

  move_group_->detachObject(identified_box);


  //Move to position goal and detach handle from end effector
  waypoints.clear();
  waypoint_cartesian_pose = position.pose;

  waypoints.push_back(waypoint_cartesian_pose);
  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

  move_group_->execute(cartesian_plan);

  move_group_->detachObject(identified_handle);


  //Detach handle in gazebo
  gazebo_link_attacher_msg.request.model_name_1 = identified_box;
  gazebo_link_attacher_msg.request.link_name_1 = identified_box + "_handle_link";
  gazebo_link_attacher_msg.request.model_name_2 = "robot";
  gazebo_link_attacher_msg.request.link_name_2 = "robot_arm_wrist_3_link";

  if(gazebo_link_detacher_client.call(gazebo_link_attacher_msg)){
    /* std:: cout << "ok" << endl; */
  };

  //Deactivate gripper
  gripper_off();
  ros::Duration(2).sleep();

  // Check if goal is active and Plan to target goal
  if (!place_on_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");

  //Plan to pre-position goal
  waypoints.clear();
  waypoint_cartesian_pose = pre_position.pose;

  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(pre_position.header.frame_id );
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan back to pre-position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //ros::Duration(3).sleep();

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
      place_feedback_.state.clear();
      place_feedback_.state = "Moved back to pre-position";
      place_on_as_->publishFeedback(place_feedback_);

      place_result_.success = true;
      place_result_.message = "Pick-up from desired position SUCCESSFUL";
      place_on_as_->setSucceeded(place_result_);
      return;
    }else{
      place_result_.success = false;
      place_result_.message = "Could not move back to pre-position";
      place_on_as_->setAborted(place_result_);
      return;
    }          
  }else{
    place_result_.success = false;
    place_result_.message = "Could not plan back to pre-position";
    place_on_as_->setAborted(place_result_);
    return;
  }

}

void ComponentSorting::gripper_on(){
  srv.request.fun = 1;
  srv.request.pin =16;
  srv.request.state =1;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 16: %s", client.getService().c_str());
     return;
   }
  srv.request.fun = 1;
  srv.request.pin =17;
  srv.request.state =1;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 17: %s", client.getService().c_str());
     return;
   }
}


void ComponentSorting::gripper_off(){
  srv.request.fun = 1;
  srv.request.pin =16;
  srv.request.state =0;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 16: %s", client.getService().c_str());
     return;
   }
  srv.request.fun = 1;
  srv.request.pin =17;
  srv.request.state =0;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 17: %s", client.getService().c_str());
     return;
   }
}

bool ComponentSorting::create_planning_scene()
{

  // Read and store parameter: objects from parameter server
  std::vector<std::string> object_names;
  bool required = true;
  readParam(pnh_, "objects", object_names, object_names, required); 

  // Create class Object_Builder objects
  for (auto & object_name: object_names)
  {
    parsed_objects.push_back(Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name));
  }

  //Object_builder objects into Moveit's collision objects


    for (auto & parsed_object: parsed_objects)
  { 
    moveit_msgs::CollisionObject collision_object;
    collision_object = parsed_object.getObject();

    try{
      transform_stamped = tfBuffer.lookupTransform("robot_base_link",collision_object.header.frame_id,ros::Time(0),ros::Duration(1.0));
    }
    catch(tf2::LookupException ex){
      ROS_ERROR("Lookup Transform error: %s", ex.what());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return false;
    }
    catch(tf2::ExtrapolationException ex){
      ROS_ERROR("Lookup Transform error: %s", ex.what());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return false;
    }
    catch(tf2::ConnectivityException ex){
      ROS_ERROR("Could not find collision object %s frame. Lookup Transform error: %s",collision_object.id.c_str(), ex.what());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return false;
    }


    collision_object.operation = collision_object.ADD;
    moveit_objects.push_back(collision_object);
  }

  // Add collision objects into the world
  planning_scene_interface.applyCollisionObjects(moveit_objects);

  return true;

}
