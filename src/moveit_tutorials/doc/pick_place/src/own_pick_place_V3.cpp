#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <ros/ros.h>


#include <moveit/trajectory_processing/iterative_time_parameterization.h>


const double tau = 2 * M_PI;


void initPose(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{ 

  geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.x = 1;
  // target_pose1.orientation.y = 0;
  // //target_pose1.orientation.z = 0;
  // //target_pose1.orientation.w = tau/8;


  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, -tau/8);

  target_pose1.orientation = tf2::toMsg(orientation);
  
  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.8;
  move_group_interface.setPoseTarget(target_pose1);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Moving to init pose %s", success ? "" : "FAILED");


  move_group_interface.move();
}

void hoverPose(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{ 

  geometry_msgs::Pose target_pose1;


  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, -tau/8);

  target_pose1.orientation = tf2::toMsg(orientation);
  
  target_pose1.position.x = 0.429;
  target_pose1.position.y = 0.1675;
  target_pose1.position.z = 0.55;
  move_group_interface.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Moving to hover pose %s", success ? "" : "FAILED");


  move_group_interface.move();
}

void pickPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction){
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.01);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose = move_group_interface.getCurrentPose().pose;
  target_pose.position.x += 0.0;
  target_pose.position.y += 0.0;
  if (direction == "down"){
    target_pose.position.z -= 0.10;
  }
  else if (direction == "up"){
    target_pose.position.z += 0.10;
  }
  waypoints.push_back(target_pose); 

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  
                                               0.0,   
                                               trajectory_msg, false);


  std::cout<< "AAAAACHTUUUUUNNNGG" << fraction << std::endl;

  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_arm");

  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 

  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  bool success = iptp.computeTimeStamps(rt, 0.1, 0.1);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  rt.getRobotTrajectoryMsg(trajectory_msg);

  std::cout << trajectory_msg << std::endl;

  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
  sleep(3.0);
 
  move_group_interface.execute(plan);  

  }


void hoverPlacePose(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{ 

  geometry_msgs::Pose target_pose1;


  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, -tau/8);

  target_pose1.orientation = tf2::toMsg(orientation);
  
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 0.8;
  move_group_interface.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  bool success = (move_group_interface.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Moving to hover place pose %s", success ? "" : "FAILED");

  move_group_interface.move();
}

void PlacePose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction)
{ 
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.01);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose = move_group_interface.getCurrentPose().pose;
  target_pose.position.x += 0.0;
  target_pose.position.y += 0.0;
  if (direction == "down"){
    target_pose.position.z -= 0.10;
  }
  else if (direction == "up"){
    target_pose.position.z += 0.10;
  }
  waypoints.push_back(target_pose); 

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  
                                               0.0,   
                                               trajectory_msg, false);

  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_arm");

 
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

 
  bool success = iptp.computeTimeStamps(rt, 0.1, 0.1);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  rt.getRobotTrajectoryMsg(trajectory_msg);

  std::cout << trajectory_msg << std::endl;

  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
  sleep(3.0);
 
  move_group_interface.execute(plan);  

}

void openHand(moveit::planning_interface::MoveGroupInterface& move_group_interface_hand)
{ 

  move_group_interface_hand.setJointValueTarget(move_group_interface_hand.getNamedTargetValues("open"));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_hand;

  bool success = (move_group_interface_hand.plan(my_plan_hand) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Open Hand %s", success ? "" : "FAILED");

 
  move_group_interface_hand.move();

}

void closeHand(moveit::planning_interface::MoveGroupInterface& move_group_interface_hand)
{ 
  
  move_group_interface_hand.setJointValueTarget(move_group_interface_hand.getNamedTargetValues("close"));
  
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_hand;

  bool success = (move_group_interface_hand.plan(my_plan_hand) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Close Hand %s", success ? "" : "FAILED");

  
  move_group_interface_hand.move();

}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.6;
  collision_objects[0].primitives[0].dimensions[1] = 1.8;
  collision_objects[0].primitives[0].dimensions[2] = 0;

  
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.45;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.01;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
 

  collision_objects[0].operation = collision_objects[0].ADD;

  
  collision_objects[1].id = "wallback";
  collision_objects[1].header.frame_id = "panda_link0";

  
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0;
  collision_objects[1].primitives[0].dimensions[1] = 1.8;
  collision_objects[1].primitives[0].dimensions[2] = 1.0;

  
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.3;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = 0.5;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  

  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[3].id = "wallleft";
  collision_objects[3].header.frame_id = "panda_link0";

  
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 1.2;
  collision_objects[3].primitives[0].dimensions[1] = 0;
  collision_objects[3].primitives[0].dimensions[2] = 1.0;

  
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.2;
  collision_objects[3].primitive_poses[0].position.y = -0.9;
  collision_objects[3].primitive_poses[0].position.z = 0.5;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  
  collision_objects[3].operation = collision_objects[3].ADD;

   
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.3;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.3;

  
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0.0;
  collision_objects[2].primitive_poses[0].position.z = 0.15;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "own_pick_place_V3");
  ros::NodeHandle nh;

 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();

 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

 
  moveit::planning_interface::MoveGroupInterface group_arm("panda_arm");
  moveit::planning_interface::MoveGroupInterface group_hand("hand");

  
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  
  const moveit::core::JointModelGroup* joint_model_group =
    group_arm.getCurrentState()->getJointModelGroup("panda_arm");
  
  
  group_arm.setPlannerId("RRTstarkConfigDefault");
  group_arm.setMaxVelocityScalingFactor(0.1);
  group_arm.setMaxAccelerationScalingFactor(0.01);
  
  
  
  addCollisionObjects(planning_scene_interface);

  
  shape_msgs::SolidPrimitive primitive;
  moveit_msgs::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.145;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.015;
  
  object_to_attach.header.frame_id = "panda_link0";
  geometry_msgs::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.x = 0.429;
  grab_pose.position.y = 0.1675;
  grab_pose.position.z = 0.55;

  
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

 
  ros::WallDuration(1.0).sleep();

  
  initPose(group_arm);
  openHand(group_hand);
  hoverPose(group_arm);
  pickPose(group_arm , "down");
  closeHand(group_hand);


  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  group_arm.attachObject("cylinder1" , "hand");
   
  
 
  pickPose(group_arm , "up");
  hoverPlacePose(group_arm);
  PlacePose(group_arm , "down");
  openHand(group_hand);
  
  
  group_arm.detachObject(object_to_attach.id);
 
  PlacePose(group_arm , "up");
  initPose(group_arm);


  ros::shutdown();
  return 0;
}