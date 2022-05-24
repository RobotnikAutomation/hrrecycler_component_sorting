#include <component_sorting_manipulation/manipulation_app.h>

ManipulationApp::ManipulationApp(ros::NodeHandle h) : RComponent(h)
{
  init(h);
}
ManipulationApp::~ManipulationApp()
{
}

void ManipulationApp::rosReadParams()
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

  moveit_constraint = "";
  readParam(pnh_, "moveit_constraint", moveit_constraint, moveit_constraint, required); 

  scale_vel_ = 1;
  readParam(pnh_, "scale_vel", scale_vel_, scale_vel_, required);

  scale_acc_ = 1;
  readParam(pnh_, "scale_acc", scale_acc_, scale_acc_, required);

  end_effector_link_ = "robot_vgc10_vgc10_link";
  readParam(pnh_, "end_effector_link", end_effector_link_, end_effector_link_, required);

  robot_base_link_ = "robot_base_footprint";
  readParam(pnh_, "robot_base_link", robot_base_link_, robot_base_link_, required);

}

int ManipulationApp::rosSetup()
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
  place_object_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PlaceObjectAction>(pnh_, "place_object", autostart));
  place_object_as_->registerGoalCallback(boost::bind(&ManipulationApp::goalCB, this, std::string("place_object")));
  place_object_as_->registerPreemptCallback(boost::bind(&ManipulationApp::preemptCB, this));
 
  pick_object_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PickObjectAction>(pnh_, "pick_object", autostart));
  pick_object_as_->registerGoalCallback(boost::bind(&ManipulationApp::goalCB, this, std::string("pick_object")));
  pick_object_as_->registerPreemptCallback(boost::bind(&ManipulationApp::preemptCB, this));
  
  // Service
  emergency_stop_trigger_ = pnh_.advertiseService("emergency_stop_trigger", &ManipulationApp::emergency_stop_cb,this);

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
  
  // TF listener
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Scene manager
  bool wait_ = true;
  scene_manager_ = std::make_unique<SceneManager>(nh_ , wait_);

  return RComponent::rosSetup();
}

int ManipulationApp::rosShutdown()
{
  return RComponent::rosShutdown();
}

int ManipulationApp::setup()
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

  // Create planning scene
  scene_manager_->initScene(); // CAMBIAR (COMPROBAR SI ES TRUE)

  return rcomponent::OK;

}

void ManipulationApp::standbyState()
{
  thread_active_flag_ = false;
  action_finished_flag_ = false;

  move_group_->clearPathConstraints();

  success_move=true;

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
  place_object_as_->start();
  RCOMPONENT_INFO_STREAM("Started action server: place_object");
  pick_object_as_->start();
  RCOMPONENT_INFO_STREAM("Started action server: pick_object");

  switchToState(robotnik_msgs::State::READY_STATE);
}

void ManipulationApp::readyState()
{ 

  if (place_object_as_->isActive() == false && pick_object_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }

  ROS_INFO_THROTTLE(3, "I have a new goal!");

  //Check which server is active and run thread
  if(place_object_as_->isActive() == true && thread_active_flag_ == false){
    thread_active_flag_ = true;
    place_object_thread_= boost::thread(&ManipulationApp::placeObject, this, place_object_goal_->object, place_object_goal_->surface);
  }   
  
  if(pick_object_as_->isActive() == true && thread_active_flag_ == false){ 
    thread_active_flag_ = true;
    pick_object_thread_= boost::thread(&ManipulationApp::pickObject, this, pick_object_goal_->object);
  }   

  if(thread_active_flag_ == true && action_finished_flag_ == true)
  {
    place_object_thread_.join();
    pick_object_thread_.join(); 
    thread_active_flag_ = false;
    action_finished_flag_ = false;
  }
  
}

void ManipulationApp::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  action_ = action;
  if(place_object_as_->isActive() || pick_object_as_->isActive()){
    ROS_INFO("Cannot process %s action, another action is active", action.c_str());
    return;
  }
  if (action_ == "place_object"){
    place_object_goal_ = place_object_as_->acceptNewGoal();
    //place_object_as_->isPreemptRequested();
  }
  if (action_ == "pick_object"){
    pick_object_goal_ = pick_object_as_->acceptNewGoal();
    //pick_object_as_->isPreemptRequested();
  }
}

void ManipulationApp::placeObject(const std::string& object, const std::string& surface)
{


  if (!place_object_as_->isActive()) return;

  // Relative pose
  geometry_msgs::Pose rel_pose;
  rel_pose.position.z = 0.5;
  rel_pose.orientation.x = -0.707;
  rel_pose.orientation.y = 0.707;

  // Move relative to crate
  if(!scene_manager_->moveRelativeTo(surface, rel_pose))
  {
    place_object_result_.success = false;
    place_object_result_.message = "Cannot move to object's place pre-position";
    if (!place_object_as_->isActive()) return;
    place_object_as_->setAborted(place_object_result_);
    thread_active_flag_ = false;
    return;
  }

  if (!place_object_as_->isActive()) return;

  //Move downwards
  geometry_msgs::PoseStamped current_pose= move_group_->getCurrentPose();
  waypoints.clear();
  waypoint_cartesian_pose = current_pose.pose;
  waypoint_cartesian_pose.position.z -= 0.5; 
  waypoints.push_back(waypoint_cartesian_pose);  

  if (!place_object_as_->isActive()) return;

  move_group_->setPoseReferenceFrame(current_pose.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);

  if (!place_object_as_->isActive()) return;

  //Detach box
  scene_manager_->detachObjects({object});

  //Detach box gazebo
  gazebo_link_attacher_msg.request.model_name_1 = object;
  gazebo_link_attacher_msg.request.link_name_1 = object + "_base_link";
  gazebo_link_attacher_msg.request.model_name_2 = "robot";
  gazebo_link_attacher_msg.request.link_name_2 = "robot_arm_wrist_3_link";
  gazebo_link_detacher_client.call(gazebo_link_attacher_msg);
  ros::Duration(1).sleep();

  // Move relative to crate
  if(!scene_manager_->moveRelativeTo(surface, rel_pose))
  {
    place_object_result_.success = false;
    place_object_result_.message = "Cannot move back to object's place pre-position";
    if (!place_object_as_->isActive()) return;
    place_object_as_->setAborted(place_object_result_);
    thread_active_flag_ = false;
    return;
  }
  
  if (!place_object_as_->isActive()) return;


  //Set succeded
  place_object_result_.success = true;
  place_object_result_.message = "Placed object on surface: " + surface;
  place_object_as_->setSucceeded(place_object_result_);

    
  action_finished_flag_ = true;

  return;
}

void ManipulationApp::pickObject(const std::string& object)
{
  // Set joint destack pre-position target
  move_group_->setNamedTarget("table_look");
  // Plan to destack pre-position target
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success_plan )
  {
    pick_object_feedback_.state.clear();
    pick_object_feedback_.state = "Plan to table look position computed";
    pick_object_as_->publishFeedback(pick_object_feedback_);
    if (!pick_object_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //Check if execute is successful
    if(success_execute){
      if (!pick_object_as_->isActive()) return;
      pick_object_feedback_.state.clear();
      pick_object_feedback_.state = "Moved to table look position";
      pick_object_as_->publishFeedback(pick_object_feedback_);
    }else{
      if (!pick_object_as_->isActive()) return;
      pick_object_result_.success = false;
      pick_object_result_.message = "Could not move to table look position";
      pick_object_as_->setAborted(pick_object_result_);
      thread_active_flag_ = false;
      return;
    }  
  }else{
    if (!pick_object_as_->isActive()) return;
    pick_object_result_.success = false;
    pick_object_result_.message = "Could not plan to table look position";
    pick_object_as_->setAborted(pick_object_result_);
    thread_active_flag_ = false;
    return;
  } 

  if (!pick_object_as_->isActive()) return;

  // Add box to frame
  scene_manager_->addObjects({object});

  // Relative pose
  geometry_msgs::Pose rel_pose;
  rel_pose.position.z = 0.4;
  rel_pose.orientation.x = -0.707;
  rel_pose.orientation.y = 0.707;

  // Move relative to crate
  if(!scene_manager_->moveRelativeTo(object, rel_pose))
  {
    pick_object_result_.success = false;
    pick_object_result_.message = "Cannot move to object's pick pre-position";
    if (!pick_object_as_->isActive()) return;
    pick_object_as_->setAborted(pick_object_result_);
    thread_active_flag_ = false;
    return;
  }

  if (!pick_object_as_->isActive()) return;

  // Relative pose
  rel_pose.position.z = 0.2;

  // Move relative to crate
  if(!scene_manager_->moveRelativeTo(object, rel_pose))
  {
    pick_object_result_.success = false;
    pick_object_result_.message = "Cannot move to object's pick position";
    if (!pick_object_as_->isActive()) return;
    pick_object_as_->setAborted(pick_object_result_);
    thread_active_flag_ = false;
    return;
  }

  if (!pick_object_as_->isActive()) return;

  // Attach crate
  scene_manager_->attachObjects({object});

  // Attach crate in gazebo
  gazebo_link_attacher_msg.request.model_name_1 = object;
  gazebo_link_attacher_msg.request.link_name_1 = object + "_base_link";
  gazebo_link_attacher_msg.request.model_name_2 = "robot";
  gazebo_link_attacher_msg.request.link_name_2 = "robot_arm_wrist_3_link";
  gazebo_link_attacher_client.call(gazebo_link_attacher_msg);
  ros::Duration(1).sleep();
 
  if (!pick_object_as_->isActive()) return;

  // Check objects underneath
  std::vector< std::string > objects_in_roi; 
  std::vector<double> dim= scene_manager_->getObjectSize(object);
  current_cartesian_pose = move_group_->getCurrentPose().pose;

  objects_in_roi = scene_manager_->getKnownObjectNamesInROI(current_cartesian_pose.position.x - dim[0]/2, current_cartesian_pose.position.y - dim[1]/2 , current_cartesian_pose.position.z - (dim[2]*2 + 0.2), current_cartesian_pose.position.x 
  + dim[0]/2, current_cartesian_pose.position.y + dim[1]/2 , current_cartesian_pose.position.z, false );

  scene_manager_->allowCollision(object,objects_in_roi);

  if (!pick_object_as_->isActive()) return;

  // Move back relative to crate

  rel_pose.position.z = 0.4;

  // Move relative to crate
  if(!scene_manager_->moveRelativeTo(object, rel_pose))
  {
    pick_object_result_.success = false;
    pick_object_result_.message = "Cannot move back to object's pick pre-position";
    if (!pick_object_as_->isActive()) return;
    pick_object_as_->setAborted(pick_object_result_);
    thread_active_flag_ = false;
    return;
  }

  if (!pick_object_as_->isActive()) return;

  // Restore collision
  scene_manager_->restoreCollision(object,objects_in_roi);

  pick_object_result_.success = true;
  pick_object_result_.message = "Picked object: " + object;
  pick_object_as_->setSucceeded(pick_object_result_);

  // Destack crate action flag
  action_finished_flag_ = true;

  return;
}


void ManipulationApp::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  move_group_->stop();
  
  if(place_object_as_->isActive()){
    place_object_result_.success = true;
    place_object_result_.message = "Place object action preempted"; 
    place_object_as_->setPreempted(place_object_result_);
    place_object_thread_.join();
    thread_active_flag_ = false;
    action_finished_flag_ = false;
  }

  if(pick_object_as_->isActive()){
    pick_object_result_.success = true;
    pick_object_result_.message = "Pick object action preempted";
    pick_object_as_->setPreempted(pick_object_result_);
    pick_object_thread_.join();
    thread_active_flag_ = false;
    action_finished_flag_ = false;
  }
  
}

bool ManipulationApp::emergency_stop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  move_group_->stop();
  res.success = true;
  res.message = "Sending stop trigger to robot";
}

