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
  // Load required parameters from ros parameter server
  bool required = true;
  bool not_required = false;

  group_name_ = "arm";
  readParam(pnh_, "group_name", group_name_, group_name_, required);

  host = "localhost";
  readParam(pnh_, "host", host, host, required);  

  port = 33829;
  readParam(pnh_, "port", port, port, required);

  double timeout = 20;
  readParam(pnh_, "move_group_timeout", timeout, timeout, not_required);
  move_group_timeout_ = ros::WallDuration(timeout);

  box.width = 0.20;
  readParam(pnh_, "box/width", box.width, box.width, required);

  box.length = 0.28;
  readParam(pnh_, "box/length", box.length, box.length, required);

  box_handle_displacement = 0.05;
  readParam(pnh_, "box_handle_displacement", box_handle_displacement, box_handle_displacement, required);

  moveit_constraint = "";
  readParam(pnh_, "moveit_constraint", moveit_constraint, moveit_constraint, required); 

  simulation = true;
  readParam(pnh_, "simulation", simulation, simulation, required); 

  multi_pin_ = false;
  readParam(pnh_, "multi_pin", multi_pin_, multi_pin_, required);	

  pin_1_ = 16;
  readParam(pnh_, "pin_1", pin_1_, pin_1_, required);	

  pin_2_ = 17;
  readParam(pnh_, "pin_2", pin_2_, pin_2_, required);	

  scale_vel_ = 1;
  readParam(pnh_, "scale_vel", scale_vel_, scale_vel_, required);

  scale_acc_ = 1;
  readParam(pnh_, "scale_acc", scale_acc_, scale_acc_, required);

  end_effector_link_ = "robot_vgc10_vgc10_link";
  readParam(pnh_, "end_effector_link", end_effector_link_, end_effector_link_, required);

  robot_link_ = "robot_base_footprint";
  readParam(pnh_, "robot_link", robot_link_, robot_link_, required);

  box_tf_watchdog_ = 5.0;
  readParam(pnh_, "box_tf_watchdog", box_tf_watchdog_, box_tf_watchdog_, not_required);
}

int ComponentSorting::rosSetup()
{
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }

  tf2_buffer_.reset(new tf2_ros::Buffer);

  // Reset move group interface
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

  // Reset planning scene interface
  bool wait = true;
  std::string name_space = "";
  planning_scene_interface_.reset(
        new moveit::planning_interface::PlanningSceneInterface(name_space, wait));

  // Reset planning scene monitor
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));      

  // Update the planning scene monitor with the current state
  bool success = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
  ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));

  // Keep up to date with new changes
  planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");

  // Reset manipulation application action servers
  bool autostart = false;
  pickup_from_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>(pnh_, "pickup_from", autostart));
  pickup_from_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("pickup_from")));
  pickup_from_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  place_on_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>(pnh_, "place_on", autostart));
  place_on_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("place_on")));
  place_on_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  move_to_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::MoveToAction>(pnh_, "move_to", autostart));
  move_to_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("move_to")));
  move_to_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  move_to_pose_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::MoveToPoseAction>(pnh_, "move_to_pose", autostart));
  move_to_pose_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("move_to_pose")));
  move_to_pose_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));

  // Service
  spawn_table = pnh_.advertiseService("spawn_table", &ComponentSorting::spawn_table_cb,this);
  set_constraint = pnh_.advertiseService("set_constraint", &ComponentSorting::set_constraint_cb,this);

  // Connect to moveit's warehouse mongo db database
  conn_ = moveit_warehouse::loadDatabase();
  conn_->setParams(host, port);

  ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);

  while (!conn_->connect())
  {
    ROS_ERROR("Failed to connect to DB on %s:%d ", host.c_str(), port);
    ros::Duration(2).sleep();
    conn_->setParams(host, port);
  }
  
  // Retrieve stored moveit motion constrains in database
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

  // Gazebo link_attacher service client
  gazebo_link_attacher_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  gazebo_link_detacher_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

  // UR vacumm gripper service client 
  gripper_client = nh_.serviceClient<ur_msgs::SetIO>("arm/ur_hardware_interface/set_io");
  // Octomap service 
  octomap_client = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");

  // TF Listener and Broadcaster
  tf_listener_ = new  tf2_ros::TransformListener(tfBuffer);

  return RComponent::rosSetup();
}

int ComponentSorting::rosShutdown()
{
  return RComponent::rosShutdown();
}

int ComponentSorting::setup()
{
  
  // Set move_group's velocity scaling factor 
  move_group_->setMaxVelocityScalingFactor(scale_vel_);
  move_group_->setMaxAccelerationScalingFactor(scale_acc_);

  // Checks if rcompnent has been correctly initialized
  int setup_result;

  setup_result = rcomponent::RComponent::setup();
  if (setup_result != rcomponent::OK)
  {
    return setup_result;
  }

  // Load positions_to_use parameter, defines the positions to be used in the manipulation application
  bool required = true;
  readParam(pnh_, "positions_to_use", positions_to_use, positions_to_use, required); 

  // Translate poses defined for each position into Pose Builder objects
  for (auto const & position: positions_to_use)
  { 
    ros::NodeHandle pnh_position_ = ros::NodeHandle(pnh_ , position);
    
    if(pnh_position_.hasParam("approach_pose")){
      approach_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_position_ , "approach_pose"))));
    }else{
      ROS_WARN("Position %s does not define an approach pose. Please fill in an appropiate pose in yaml file if required", position.c_str());
    }

    if(pnh_position_.hasParam("pre_place_pose")){
      pre_place_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_position_ , "pre_place_pose"))));
    }else{
      ROS_WARN("Position %s does not define a pre-place pose. Please fill in an appropiate pose in yaml file if required", position.c_str());
    }

    if(pnh_position_.hasParam("place_pose")){
      place_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_position_ , "place_pose"))));
    }else{
      ROS_WARN("Position %s does not define a place pose. Please fill in an appropiate pose in yaml file if required", position.c_str());
    }

    if(pnh_position_.hasParam("pre_pick_pose")){
      pre_pick_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_position_ , "pre_pick_pose"))));
    }else{
      ROS_WARN("Position %s does not define a pre-pick pose. Please fill in an appropiate pose in yaml file if required", position.c_str());
    }

    if(pnh_position_.hasParam("pick_pose")){
      pick_poses_.insert(make_pair(position, Pose_Builder(ros::NodeHandle(pnh_position_ , "pick_pose"))));
    }else{
      ROS_WARN("Position %s does not define a pick pose. Please fill in an appropiate pose in yaml file if required", position.c_str());
    }

  }

  // Create planning scene
  if(create_planning_scene()){
    return rcomponent::OK;
  }else{
    return rcomponent::ERROR;
  }
}

void ComponentSorting::standbyState()
{
  // Move to home position without selected constraints
  move_group_->detachObject();  
  move_group_->clearPathConstraints();
  move_group_->setNamedTarget("home");
  success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success_move){
    // Select constraint and check whether it exists
    move_group_->setPathConstraints(moveit_constraint);

    current_constraint = move_group_->getPathConstraints();
    if(moveit_constraint != current_constraint.name){
      ROS_ERROR("Desired moveit_constraint: %s is not available in database, please modify or run generate_path_constraints.cpp", moveit_constraint.c_str());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return;
    }

    ROS_INFO("Planning with constraint: %s",moveit_constraint.c_str());

    // Start manipulation application action servers
    pickup_from_as_->start();
    RCOMPONENT_INFO_STREAM("Started server: pickup from");
    place_on_as_->start();
    RCOMPONENT_INFO_STREAM("Started server: place on");
    move_to_as_->start();
    RCOMPONENT_INFO_STREAM("Started server: move to");
    move_to_pose_as_->start();
    RCOMPONENT_INFO_STREAM("Started server: move to pose");

    ROS_INFO("Moved to home position, ready to take commands");

    switchToState(robotnik_msgs::State::READY_STATE);

  }else{
    ROS_WARN("Could not move to home position");
    switchToState(robotnik_msgs::State::FAILURE_STATE);
  }

}

void ComponentSorting::readyState()
{ 

  if (pickup_from_as_->isActive() == false && place_on_as_->isActive() == false && move_to_as_->isActive() == false && move_to_pose_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }

  ROS_INFO_THROTTLE(3, "I have a new goal!");

  //Check which server is active 
  if(pickup_from_as_->isActive() == true){ 

    // Get desired pickup from goal position
    std::string pick_position = pickup_from_goal_->from;
    //Check if position exists
    if(find(positions_to_use.begin(), positions_to_use.end(),pick_position) != end(positions_to_use)){
      pick_chain_movement(pick_position);
    }else {
      ROS_WARN("Position %s does not exist in poses config yaml", pick_position.c_str());
            
      pick_result_.success = false;
      pick_result_.message = "Position does not exist in poses config yaml";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }
    
  }   
  
  if(place_on_as_->isActive() == true){ 
    // Get desired place on goal position
    std::string place_position = place_on_goal_->in;

    //Check if position exists
    if(find(positions_to_use.begin(), positions_to_use.end(),place_position) != end(positions_to_use)){
      place_chain_movement(place_position);
    }else {
      ROS_WARN("Position %s does not exist in poses config yaml", place_position.c_str());

      place_result_.success = false;
      place_result_.message = "Position does not exist in poses config yaml";
      place_on_as_->setAborted(place_result_);
      return;
    }
    
  }   

  if(move_to_as_->isActive() == true){ 
    // Get desired goal and set as target
    std::string move_to_position = move_to_goal_->to;
    // Call move_to function
    move_to(move_to_position);
  } 

  if(move_to_pose_as_->isActive() == true){ 
    // Get desired goal and set as target
    geometry_msgs::PoseStamped pose = move_to_pose_goal_->pose;
    // Call move_to_pose function
    move_to_pose(pose);
  } 
}

void ComponentSorting::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  action_ = action;
  if(pickup_from_as_->isActive() || place_on_as_->isActive() || move_to_as_->isActive() || move_to_pose_as_->isActive()){
    ROS_INFO("Cannot process %s action, another action is active", action.c_str());
    return;
  }
  if (action_ == "pickup_from"){
    pickup_from_goal_ = pickup_from_as_->acceptNewGoal();
    pickup_from_as_->isPreemptRequested();
  }
  if (action_ == "place_on"){
    place_on_goal_ = place_on_as_->acceptNewGoal();
    place_on_as_->isPreemptRequested();
  }
  if (action_ == "move_to"){
    move_to_goal_ = move_to_as_->acceptNewGoal();
    move_to_as_->isPreemptRequested();
  }    
  if (action_ == "move_to_pose"){
    move_to_pose_goal_ = move_to_pose_as_->acceptNewGoal();
    move_to_pose_as_->isPreemptRequested();
  }   
}

void ComponentSorting::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  move_group_->stop();
  pickup_from_as_->setPreempted();
  place_on_as_->setPreempted();
  move_to_as_->setPreempted();
  move_to_pose_as_->setPreempted();
}

bool ComponentSorting::spawn_table_cb(component_sorting_msgs::SpawnTable::Request &req, component_sorting_msgs::SpawnTable::Response &res)
{
  std::vector<moveit_msgs::CollisionObject> add_collision_objects_;
  moveit_msgs::CollisionObject table_1 = parsed_objects_.at("table_1").getObject();
  moveit_msgs::CollisionObject table_2 = parsed_objects_.at("table_2").getObject();

  if (req.table == "table_1"){
    table_1.operation = table_1.ADD;
    table_2.operation = table_2.REMOVE;
  }
  else if(req.table == "table_2"){
    table_2.operation = table_2.ADD;
    table_1.operation = table_1.REMOVE;
  }
  else{
  ROS_ERROR("%s can not be spawned, it is not defined in the YAML", req.table.c_str());
  res.result = false;
  return 0;
  }
  res.result = true;
  add_collision_objects_.push_back(table_1);
  add_collision_objects_.push_back(table_2);

  planning_scene_interface_->applyCollisionObjects(add_collision_objects_);

  return 1;
}

bool ComponentSorting::set_constraint_cb(component_sorting_msgs::SetConstraint::Request &req, component_sorting_msgs::SetConstraint::Response &res)
{
  //Set constraint
  current_constraint = move_group_->getPathConstraints();

  if(current_constraint.name == "move_parallel")
  {
    ROS_ERROR("Move_parallel constraint currently in use, cannot change constraint");
    res.result = false;
    return 0;
  }

  bool check_constraint;
  if(req.constraint.empty())
  {
    move_group_->clearPathConstraints();
    check_constraint = true;
  }
  else
  {
    check_constraint = move_group_->setPathConstraints(req.constraint);
  }

  res.result = check_constraint;
  return check_constraint;
}



void ComponentSorting::move_to_pose(geometry_msgs::PoseStamped pose){
  
  // Set pre-position goal
  move_group_->setPoseTarget(pose);
  // Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // If plan is successful execute trajectory
  if(success_plan){
    move_to_pose_feedback_.state.clear();
    move_to_pose_feedback_.state = "Plan to desired position computed";
    move_to_pose_as_->publishFeedback(move_to_pose_feedback_);

    //Check if goal is active and move to pre-position goal
    if (!move_to_pose_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      move_to_pose_feedback_.state.clear();
      move_to_pose_feedback_.state = "Moved end-effector to desired pose";
      move_to_pose_as_->publishFeedback(move_to_pose_feedback_);

      move_to_pose_result_.success = true;
      move_to_pose_result_.message = "Move end-effector to desired pose action: SUCCESSFUL";
      move_to_pose_as_->setSucceeded(move_to_pose_result_);
      return;
    }else{
      move_to_pose_result_.success = false;
      move_to_pose_result_.message = "Could not move end-effector to desired pose";
      move_to_pose_as_->setAborted(move_to_pose_result_);
      return;
    }
          
  }else{
    move_to_pose_result_.success = false;
    move_to_pose_result_.message = "Could not plan to desired pose";
    move_to_pose_as_->setAborted(move_to_pose_result_);
    return;
  }
}

void ComponentSorting::move_to(std::string move_to_position)
{ 
  // Look if position is defined in poses yaml file or in srdf
  if(find(positions_to_use.begin(), positions_to_use.end(), move_to_position) != end(positions_to_use)){

    // Initialize required poses
    geometry_msgs::PoseStamped approach_position;

    // Extract poses from available lists:
    try{
      approach_position = approach_poses_.at(move_to_position).getPose();
    }catch (const std::out_of_range& e){
      move_to_result_.success = false;
      move_to_result_.message = "Cannot perform move_to action, please make sure all required poses (approach) are defined for the selected move to position";
      move_to_as_->setAborted(move_to_result_);
      ROS_ERROR(move_to_result_.message.c_str());
      return;
    } 

    // Set pre-position goal
    move_group_->setPoseTarget(approach_position);

  }else {
    // Look if position is defined in srdf
    if(!move_group_->setNamedTarget(move_to_position)){
      move_to_result_.success = false;
      move_to_result_.message = "Position does not exist, it is not defined in poses yaml or srdf.";
      move_to_as_->setAborted(move_to_result_);
      ROS_ERROR(move_to_result_.message.c_str());
      return;
    }
  }

  // Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // If plan is successful execute trajectory
  if(success_plan){
    move_to_feedback_.state.clear();
    move_to_feedback_.state = "Plan to desired position computed";
    move_to_as_->publishFeedback(move_to_feedback_);

    //Check if goal is active and move to pre-position goal
    if (!move_to_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      move_to_feedback_.state.clear();
      move_to_feedback_.state = "Moved end-effector to desired position";
      move_to_as_->publishFeedback(move_to_feedback_);

      move_to_result_.success = true;
      move_to_result_.message = "Move end-effector to desired position action: SUCCESSFUL";
      move_to_as_->setSucceeded(move_to_result_);
      return;
    }else{
      move_to_result_.success = false;
      move_to_result_.message = "Could not move end-effector to desired position";
      move_to_as_->setAborted(move_to_result_);
      return;
    }
          
  }else{
    move_to_result_.success = false;
    move_to_result_.message = "Could not plan to desired position";
    move_to_as_->setAborted(move_to_result_);
    return;
  }
}

void ComponentSorting::pick_chain_movement(std::string pick_position)
{  
  // Initialize poses required to perform pickup from action
  geometry_msgs::PoseStamped pre_pick_pose, pick_pose;

  // Extract required poses from available lists
  try{
    pre_pick_pose = pre_pick_poses_.at(pick_position).getPose();
    pick_pose = pick_poses_.at(pick_position).getPose();
  }catch (const std::out_of_range& e){
    pick_result_.success = false;
    pick_result_.message = "Cannot perform pickup_from action, please make sure all required poses (pre-pick and pick) are defined for the selected position";
    pickup_from_as_->setAborted(pick_result_);
    ROS_WARN(pick_result_.message.c_str());
    return;
  } 

  // Process box and handle collision objects
  try{
    box_ = parsed_objects_.at("box").getObject();
    handle_ = parsed_objects_.at("handle").getObject();
  }catch (const std::out_of_range& e){
    pick_result_.success = false;
    pick_result_.message = "Cannot perform pickup_from action, please define box and handle objects in yaml file.";
    pickup_from_as_->setAborted(pick_result_);
    ROS_WARN(pick_result_.message.c_str());
    return;
  }     

  box_.operation = box_.ADD;
  handle_.operation = handle_.ADD;

  // Fill in box and handle collision object frame id
  box_.header.frame_id = pick_pose.header.frame_id;
  handle_.header.frame_id = pick_pose.header.frame_id;

  // Check whether frame identified by box visual detection algorithm is updated
  try{
    transform_stamped = tfBuffer.lookupTransform(robot_link_,box_.header.frame_id,ros::Time::now(),ros::Duration(box_tf_watchdog_));
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("Did not receive updated detected box frame.");
    pick_result_.success = false;
    pick_result_.message = "Cannot perform pickup_from action, did not receive an updated detected box frame.";
    pickup_from_as_->setAborted(pick_result_);
    ROS_WARN(pick_result_.message.c_str());
    return;
  }

  // Add box and handle to frame identified by box visual detection algorithm
  std::vector<moveit_msgs::CollisionObject> add_collision_objects_;
  add_collision_objects_.push_back(box_);
  add_collision_objects_.push_back(handle_);
  planning_scene_interface_->applyCollisionObjects(add_collision_objects_);

  // Clear octomap
  std_srvs::Empty octomap_msg;
  octomap_client.call(octomap_msg);

  // Set pre-pick pose as goal and compute plan
  move_group_->setPoseTarget(pre_pick_pose);
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
 
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
  if (!pickup_from_as_->isActive()) return;
  
  // Get current end effector position and check whether there is a moveit collision box to grab
  current_cartesian_pose = move_group_->getCurrentPose().pose;

  objects_in_roi.clear();
  objects_in_roi = planning_scene_interface_->getKnownObjectNamesInROI(current_cartesian_pose.position.x - box.length/2, current_cartesian_pose.position.y - box.width/2 ,0, current_cartesian_pose.position.x 
  + box.length/2, current_cartesian_pose.position.y + box.width/2 , 3, false );

  for(int i=0; i < objects_in_roi.size(); i++){
   if(objects_in_roi[i].compare(0,6,"handle")==0){
     identified_handle = objects_in_roi[i];
   }else if (objects_in_roi[i].compare(0,3,"box")==0) {
      identified_box = objects_in_roi[i];
   }else {}
  }

  if(identified_handle.empty() || identified_box.empty()){
    pick_result_.success = false;
    pick_result_.message = "There is no moveit collision box and handle to attach.";
    pickup_from_as_->setAborted(pick_result_);
    ROS_WARN(pick_result_.message.c_str());
    return;
  }

  //Allow contact between end effector and handle
  acm_ = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
  acm_.setEntry(end_effector_link_, identified_handle, true);
  acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
  planning_scene_msg.is_diff = true;
  planning_scene_interface_->applyPlanningScene(planning_scene_msg);

  // Move to pick pose
  move_group_->setPoseTarget(pick_pose);
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //If plan is successful execute trajectory
  if(success_plan){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan to desired position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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

  // Select move_parallel constraint
  move_group_->setPathConstraints("move_parallel");
  current_constraint = move_group_->getPathConstraints();

  if(current_constraint.name != "move_parallel"){
    pick_result_.success = false;
    pick_result_.message = "Moveit move_parallel constraint not available in database";
    pickup_from_as_->setAborted(pick_result_);
  }

  //Attach moveit collision object identified as handle
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

  //Activate gripper or pick handle in Gazebo
  if(simulation){
    //Pick handle in gazebo
    gazebo_link_attacher_msg.request.model_name_1 = identified_box;
    gazebo_link_attacher_msg.request.link_name_1 = identified_box + "_handle_link";
    gazebo_link_attacher_msg.request.model_name_2 = "robot";
    gazebo_link_attacher_msg.request.link_name_2 = "robot_arm_wrist_3_link";
    gazebo_link_attacher_client.call(gazebo_link_attacher_msg);
  }else{
    //Activate gripper
    gripper_on();
    ros::Duration(0.5).sleep();
  }

  //Allow contact between attached box and holder
  acm_ = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
  acm_.setEntry(pick_position + "_holder_link", identified_box, true);
  acm_.setEntry("safety_box_handle", identified_handle, true);
  acm_.setEntry("safety_box_handle", identified_box, true);
  acm_.setEntry("safety_box", identified_handle, true);
  acm_.setEntry("safety_box", identified_box, true);
  acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
  planning_scene_msg.is_diff = true;
  planning_scene_interface_->applyPlanningScene(planning_scene_msg);

  //Move 5cm upwards and attach box
  geometry_msgs::PoseStamped current_pose= move_group_->getCurrentPose();
  waypoints.clear();
  waypoint_cartesian_pose = current_pose.pose;
  waypoint_cartesian_pose.position.z += box_handle_displacement; 
  waypoints.push_back(waypoint_cartesian_pose);  

  move_group_->setPoseReferenceFrame(current_pose.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);

  //Attach moveit collision object identified as box
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


  // Move back to pre-pick pose
  move_group_->setPoseReferenceFrame(pre_pick_pose.header.frame_id);
  waypoints.clear();
  waypoint_cartesian_pose = pre_pick_pose.pose;
  waypoints.push_back(waypoint_cartesian_pose);  
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

  //Restore collision checking between box to attach and holder
  acm_ = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
  acm_.setEntry(pick_position + "_holder_link", identified_box, false);
  acm_.setEntry("safety_box_handle", identified_handle, false);
  acm_.setEntry("safety_box_handle", identified_box, false);
  acm_.setEntry("safety_box", identified_handle, false);
  acm_.setEntry("safety_box", identified_box, false);
  acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
  planning_scene_msg.is_diff = true;
  planning_scene_interface_->applyPlanningScene(planning_scene_msg);


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
  // Initialize required poses
    geometry_msgs::PoseStamped pre_place_pose;
    geometry_msgs::PoseStamped place_pose; 

  // Search required poses from available ones
  try{
    pre_place_pose = pre_place_poses_.at(place_position).getPose();
    place_pose = place_poses_.at(place_position).getPose();
  }catch (const std::out_of_range& e){
    place_result_.success = false;
    place_result_.message = "Cannot perform place_on action, please make sure all required poses (pre-place and place) are defined for the selected place in position";
    place_on_as_->setAborted(place_result_);
    ROS_WARN(place_result_.message.c_str());
    return;
  } 

  // Check if we are holding a box
  std::map< std::string, moveit_msgs::AttachedCollisionObject > attached_objects_map = planning_scene_interface_->getAttachedObjects();
  std::vector< std::string> attached_objects;
  for(auto const & object : attached_objects_map)
  {
    attached_objects.push_back(object.first);
  }

  if(attached_objects.empty()){
    ROS_WARN("There is no box to detach");
    place_result_.success = false;
    place_result_.message = "There is no box to detach";
    place_on_as_->setAborted(place_result_);
    return;
  }

  // Set pre-position goal and compute plan
  move_group_->setPoseTarget(pre_place_pose);
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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

  //Allow contact between attached box and holder
  acm_ = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
  acm_.setEntry(place_position + "_holder_link", identified_box, true);
  acm_.setEntry("safety_box_handle", identified_handle, true);
  acm_.setEntry("safety_box_handle", identified_box, true);
  acm_.setEntry("safety_box", identified_handle, true);
  acm_.setEntry("safety_box", identified_box, true);
  acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
  //acm_.print(std::cout);
  planning_scene_msg.is_diff = true;
  planning_scene_interface_->applyPlanningScene(planning_scene_msg);

  //Plan to position goal 
  move_group_->setPoseReferenceFrame(place_pose.header.frame_id);
  waypoints.clear();
  waypoint_cartesian_pose = place_pose.pose;
  waypoints.push_back(waypoint_cartesian_pose); 
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

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
      acm_.setEntry(place_position + "_holder_link", identified_box, false);
      acm_.setEntry("safety_box_handle", identified_handle, false);
      acm_.setEntry("safety_box_handle", identified_box, false);
      acm_.setEntry("safety_box", identified_handle, false);
      acm_.setEntry("safety_box", identified_box, false);
      acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
      planning_scene_msg.is_diff = true;
      planning_scene_interface_->applyPlanningScene(planning_scene_msg);
      return;
    }
          
  }else{
    place_result_.success = false;
    place_result_.message = "Could not plan to desired position";
    place_on_as_->setAborted(place_result_);
    acm_.setEntry(place_position + "_holder_link", identified_box, false);
    acm_.setEntry("safety_box_handle", identified_handle, false);
    acm_.setEntry("safety_box_handle", identified_box, false);
    acm_.setEntry("safety_box", identified_handle, false);
    acm_.setEntry("safety_box", identified_box, false);
    acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
    planning_scene_msg.is_diff = true;
    planning_scene_interface_->applyPlanningScene(planning_scene_msg);
    return;
  }

  //Detach box from end effector
  move_group_->detachObject(identified_box);

  //Move x cm downwards and detach handle from end effector
  geometry_msgs::PoseStamped current_pose= move_group_->getCurrentPose();
  waypoints.clear();
  waypoint_cartesian_pose = current_pose.pose;
  waypoint_cartesian_pose.position.z -= box_handle_displacement; 
  waypoints.push_back(waypoint_cartesian_pose);  

  move_group_->setPoseReferenceFrame(current_pose.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);

  move_group_->detachObject(identified_handle);


  //Deactivate gripper or detach handle in Gazebo
  if(simulation){
    //Detach handle in gazebo
    gazebo_link_attacher_msg.request.model_name_1 = identified_box;
    gazebo_link_attacher_msg.request.link_name_1 = identified_box + "_handle_link";
    gazebo_link_attacher_msg.request.model_name_2 = "robot";
    gazebo_link_attacher_msg.request.link_name_2 = "robot_arm_wrist_3_link";
   gazebo_link_detacher_client.call(gazebo_link_attacher_msg);
  }else{
    //Deactivate gripper
    gripper_off();
    ros::Duration(0.5).sleep();
  };

  // Restore selected constraint instead of move_parallel
  move_group_->setPathConstraints(moveit_constraint);

  // Check if goal is active and Plan to target goal
  if (!place_on_as_->isActive()) return;

  //Plan to pre-position goal
  waypoints.clear();
  waypoint_cartesian_pose = pre_place_pose.pose;

  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(pre_place_pose.header.frame_id );
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

  // Restore collision checking between end effector and handle
  acm_ = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix();
  acm_.setEntry(end_effector_link_, identified_handle, false);
  acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
  planning_scene_msg.is_diff = true;
  planning_scene_interface_->applyPlanningScene(planning_scene_msg);

  // Remove box collision object from moveit 
  box_.operation = box_.REMOVE;
  handle_.operation = handle_.REMOVE;

  std::vector<moveit_msgs::CollisionObject> add_collision_objects_;
  add_collision_objects_.push_back(box_);
  add_collision_objects_.push_back(handle_);
  planning_scene_interface_->applyCollisionObjects(add_collision_objects_);

  //Restore collision checking between attached box and holder
  acm_.setEntry(place_position + "_holder_link", identified_box, false);
  acm_.setEntry("safety_box_handle", identified_handle, false);
  acm_.setEntry("safety_box_handle", identified_box, false);
  acm_.setEntry("safety_box", identified_handle, false);
  acm_.setEntry("safety_box", identified_box, false);
  acm_.getMessage(planning_scene_msg.allowed_collision_matrix);
  planning_scene_msg.is_diff = true;
  planning_scene_interface_->applyPlanningScene(planning_scene_msg);


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
      place_result_.message = "Place-on from desired position SUCCESSFUL";
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
  srv.request.pin = pin_1_;
  srv.request.state =1;
  if (gripper_client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin %d: %s",pin_1_, gripper_client.getService().c_str());
     return;
   }
  srv.request.fun = 1;
  srv.request.pin = pin_2_;
  srv.request.state =1;
  if(multi_pin_){	
    if (gripper_client.call(srv))
     {
       ROS_INFO("Result: %d", srv.response.success);
     }
    else
     {
       ROS_ERROR("Failed to call service set/IO on pin %d: %s", pin_2_, gripper_client.getService().c_str());
       return;
     }
  } 
}


void ComponentSorting::gripper_off(){

  srv.request.fun = 1;
  srv.request.pin = pin_1_;
  srv.request.state =0;
  if (gripper_client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin %d: %s",pin_1_, gripper_client.getService().c_str());
     return;
   }
  srv.request.fun = 1;
  srv.request.pin = pin_2_;
  srv.request.state =0;
  if(multi_pin_){	
    if (gripper_client.call(srv))
     {
       ROS_INFO("Result: %d", srv.response.success);
     }
    else
     {
       ROS_ERROR("Failed to call service set/IO on pin %d: %s", pin_2_, gripper_client.getService().c_str());
       return;
     }
  }
}

bool ComponentSorting::create_planning_scene()
{

  // Read and store parameter: objects from parameter server
  std::vector<std::string> object_names;
  bool required = true;
  readParam(pnh_, "objects", object_names, object_names, required); 

  // Create class Object_Builder objects
  for (auto const & object_name: object_names)
  {
    //parsed_objects.push_back(Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name));
    parsed_objects_.insert(make_pair(object_name, Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name)));  
  }

  //Object_builder objects into Moveit's collision objects


  for (auto & [id, parsed_object] : parsed_objects_)
  { 
    if(parsed_object.getSpawn()){
      moveit_msgs::CollisionObject collision_object;
      collision_object = parsed_object.getObject();

    try{
        transform_stamped = tfBuffer.lookupTransform(robot_link_,collision_object.header.frame_id,ros::Time(0),ros::Duration(1.0));
      }
      catch(tf2::TransformException ex){
        ROS_ERROR("Error when adding %s object to desired frame. Lookup Transform error: %s",collision_object.id.c_str(), ex.what());
        switchToState(robotnik_msgs::State::FAILURE_STATE);
        return false;
      }


      collision_object.operation = collision_object.ADD;
      moveit_objects.push_back(collision_object);
    }
  }

  // Add collision objects into the world
  planning_scene_interface_->applyCollisionObjects(moveit_objects);

  return true;

}
