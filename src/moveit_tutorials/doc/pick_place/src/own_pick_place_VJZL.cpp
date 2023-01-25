#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS
#include <ros/ros.h>

//Time Parametrization
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/Bool.h"
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


//Functions for Moving and grasping with robot
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion initPose
class pick_place{

  public:
  pick_place():
            nh{},
            pub(nh.advertise<std_msgs::Bool>("goal_state", 10)){}
            //sub(nh.subscribe("topic", 1000, &pick_place::callback, this)),
            //timer(nh.createTimer(ros::Duration(0.1), &pick_place::main_loop, this)){}
  
void initPose(moveit::planning_interface::MoveGroupInterface& move_group)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose_init;

  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, -tau/8);

  target_pose_init.orientation = tf2::toMsg(orientation);
  
  target_pose_init.position.x = 0.5;
  target_pose_init.position.y = 0;
  target_pose_init.position.z = 0.7;
  move_group.setPoseTarget(target_pose_init);

  move_group.move();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion hoverPose

int hoverPose(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose_hover;

  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, -tau/8);

  target_pose_hover.orientation = tf2::toMsg(orientation);
  
  target_pose_hover.position.x = x;
  target_pose_hover.position.y = y;
  target_pose_hover.position.z = 0.6; 
  move_group.setPoseTarget(target_pose_hover);

  move_group.move();
  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion pickPose as Cartesian Motion

void pickPose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction){
  moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose_pick = move_group_interface.getCurrentPose().pose;
  target_pose_pick.position.x += 0.0;
  target_pose_pick.position.y += 0.0;
  if (direction == "down"){
    target_pose_pick.position.z -= 0.055;
  }
  else if (direction == "up"){
    target_pose_pick.position.z += 0.055;
  }
  waypoints.push_back(target_pose_pick); 

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
  
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
  
  // Modify trajectory for adjusting speed
  
  // Create robot trajectory object
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_arm");

  // Get robot trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  //Compute TimeStamps
  iptp.computeTimeStamps(rt, 0.1, 0.1);
  
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  cartesianPlan.trajectory_ = trajectory_msg;
 
  move_group_interface.execute(cartesianPlan);  

  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion hoverPlacePose

void hoverPlacePose(moveit::planning_interface::MoveGroupInterface& move_group, float x, float y)
{ 
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose pose_hover_place;
  std_msgs::Bool status;

  status.data = true;
  pub.publish(status);
  //Convert Orienation from RPY to Quaternion
  tf2::Quaternion orientation;
  orientation.setRPY( -tau/2, 0, -tau/8);

  pose_hover_place.orientation = tf2::toMsg(orientation);
  
  pose_hover_place.position.x = x;
  pose_hover_place.position.y = y;
  pose_hover_place.position.z = 0.6;
  move_group.setPoseTarget(pose_hover_place);

  move_group.move();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Position, Orientation, Planning, Exectuion PlacePose

void PlacePose(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::string direction)
{ 
  moveit::planning_interface::MoveGroupInterface::Plan cartesianPlan;
  move_group_interface.setStartStateToCurrentState();

  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose_place = move_group_interface.getCurrentPose().pose;
  target_pose_place.position.x += 0.0;
  target_pose_place.position.y += 0.0;
  if (direction == "down"){
    target_pose_place.position.z -= 0.05;
  }
  else if (direction == "up"){
    target_pose_place.position.z += 0.05;
  }
  waypoints.push_back(target_pose_place); 

  moveit_msgs::RobotTrajectory trajectory_msg;
  move_group_interface.setPlanningTime(10.0);
 
  double fraction = move_group_interface.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
  // Modify trajectory for adjusting speed
  
  // Create robot trajectory object
  robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "panda_arm");

  // Get robot trajectory
  rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);
 
  // Create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;

  //Compute TimeStamps
  iptp.computeTimeStamps(rt, 0.1, 0.1);
  
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  cartesianPlan.trajectory_ = trajectory_msg;
 
 
  move_group_interface.execute(cartesianPlan);  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// close Gripper for moveit msg grasp

void closedGripper(trajectory_msgs::JointTrajectory& posture, float y)
{

  // Add both finger joints
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  //set closed position
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  float f1;
  float f2;
  float e1;
  float e2;
  if(y==0){
    float f1 = 0.0006;
    float f2 = 0.0006;
    float e1 = 400;
    float e2 = 400;
  }
  else if(y == 0.28){
    float f1 = 0.001;
    float f2 = 0.001;
    float e1 = 150;
    float e2 = 150;
  }
  else
  {
    float f1 = 0.001;
    float f2 = 0.001;
    float e1 = 200;
    float e2 = 200;
  }
  posture.points[0].positions[0] = f1;
  posture.points[0].positions[1] = f2;
  posture.points[0].effort.resize(2);
  posture.points[0].effort[0] = e1;
  posture.points[0].effort[1] = e2;
  
  //Additonal try if force is possible
  //posture.points[0].effort.resize(1);
  //posture.points[0].effort[0] = 1;
  
  
  posture.points[0].time_from_start = ros::Duration(0.5);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Little movement for grasping with grasp msg

void pick(moveit::planning_interface::MoveGroupInterface& move_group, float y)
{
  //Create Vector for grasp approaches (only need 1)
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";

  // Setting grasp pose
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  grasps[0].grasp_pose = current_pose;
  grasps[0].grasp_pose.pose.position.z -= 0.001;
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
 
  //std::cout << "------------Pick Pose----------" << std::endl;
  //std::cout << current_pose << std::endl;

  // Setting pre-grasp approach

  // Direction is set as negative z axis, approach from above the object
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.001;
  grasps[0].pre_grasp_approach.desired_distance = 0.002;

  //Close gripper to grasp object
  closedGripper(grasps[0].grasp_posture, y);

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("cylinder1", grasps);
  }

///////////////////////////////////////////////////////////////////////////////////////
// Plan and execute open hand

void openHand(moveit::planning_interface::MoveGroupInterface& move_group_interface_hand)
{ 
  // Open the gripper
  move_group_interface_hand.setJointValueTarget(move_group_interface_hand.getNamedTargetValues("open"));

  //Move the robot
  move_group_interface_hand.move();

}

//planning scene
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Objects for planing scene including table, wall back, wall left


void main_loop(){
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //planning interface
  moveit::planning_interface::MoveGroupInterface group_arm("panda_arm");
  moveit::planning_interface::MoveGroupInterface group_hand("hand");
  //  group_arm.setPlanningTime(45.0);
  // Set parameters for group like planner, speed, acceleration
  group_arm.setPlannerId("RRTConnect");
  group_arm.setMaxVelocityScalingFactor(0.02);
  group_arm.setMaxAccelerationScalingFactor(0.02);
  float pick_x[7] = {0.228, 0.376, 0.228, 0.232, 0.375, 0.22, 0.375};
  float pick_y[7] = {0.28, 0.278, 0.42, 0.1, 0.1, 0, 0.45};
  float place_x[7] = {0.225, 0.376, 0.232, 0.232, 0.375, 0.3, 0.375};
  float place_y[7] = {-0.302, -0.290, -0.42, -0.1, -0.1, 0, -0.45};
  int i = 0;
  for (int i = 0; i < 7;i = i + 1)

    // Add Objects to the envoirement
    //addCollisionObjects(planning_scene_interface);

    //Create Cylinder
    // shape_msgs::SolidPrimitive primitive;

    // moveit_msgs::CollisionObject object_to_attach;
    // object_to_attach.id = "cylinder1";

    // shape_msgs::SolidPrimitive cylinder_primitive;
    // cylinder_primitive.type = primitive.CYLINDER;
    // cylinder_primitive.dimensions.resize(2);
    // cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.145;
    // cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.013;
    
    // // define the frame/pose for this cylinder
    // object_to_attach.header.frame_id = "panda_link0";
    // geometry_msgs::Pose grab_pose;
    // grab_pose.orientation.w = 1.0;
    // grab_pose.position.x = 0.429;
    // grab_pose.position.y = 0.1675;
    // grab_pose.position.z = 0.41;

    // // First, we add the object to the world (without using a vector)
    // object_to_attach.primitives.push_back(cylinder_primitive);
    // object_to_attach.primitive_poses.push_back(grab_pose);
    // object_to_attach.operation = object_to_attach.ADD;
    // planning_scene_interface.applyCollisionObject(object_to_attach);

    // // Wait a bit for ROS things to initialize
    // ros::WallDuration(1.0).sleep();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Start motion to each position grasp, release and detach
    ROS_WARN("round %d start at:%.8f",i,ros::Time::now().toSec());
    ROS_WARN("--------------------");

    //First open Hand, move to init pose
    //ROS_WARN("openhand start at:%.8f",ros::Time::now().toSec());
    //ROS_WARN("--------------------");
    openHand(group_hand);
    //ROS_WARN("openhand end at:%.8f",ros::Time::now().toSec());
    //ROS_WARN("--------------------");


    // ROS_WARN("initpose start at:%.8f",ros::Time::now().toSec());
    // ROS_WARN("--------------------");
    // initPose(group_arm);
    // ROS_WARN("initpose end at:%.8f",ros::Time::now().toSec());
    // ROS_WARN("--------------------");

    //Move above holder and down for picking object and up again
    ROS_WARN("pickhover start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    hoverPose(group_arm, pick_x[i], pick_y[i]);
    ROS_WARN("pickhover end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------"); 
    //ros::WallDuration(2.0).sleep();

    ROS_WARN("pickdown start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    pickPose(group_arm , "down");
    ROS_WARN("pickdown end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    //ros::WallDuration(2.0).sleep();

    ROS_WARN("closehand start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    pick(group_arm, pick_y[i]);
    ROS_WARN("closehand end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    //ros::WallDuration(2.0).sleep();

    ROS_WARN("pickup start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    pickPose(group_arm , "up");
    ROS_WARN("pickup end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    //ros::WallDuration(2.0).sleep();
    //initPose(group_arm);
    //Move to placing pose and place object
    ROS_WARN("placehover start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    hoverPlacePose(group_arm, place_x[i], place_y[i]);
    ROS_WARN("placehover end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    //ros::WallDuration(3.0).sleep();

    ROS_WARN("hover start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    PlacePose(group_arm , "down");
    ROS_WARN("hover end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    //ros::WallDuration(3.0).sleep();

    ROS_WARN("losehand start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    openHand(group_hand);
    ROS_WARN("losehand end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    //ros::WallDuration(3.0).sleep();

    // detach the cylinder from the robot's gripper.
    // ROS_WARN("detachObject start at:%.8f",ros::Time::now().toSec());
    // ROS_WARN("--------------------");
    //group_arm.detachObject(object_to_attach.id);
    // ROS_WARN("detachObject end at:%.8f",ros::Time::now().toSec());
    // ROS_WARN("--------------------");

    // Move up and to init pose
    ROS_WARN("Placeup start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    PlacePose(group_arm , "up");
    ROS_WARN("Placeup end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    //ros::WallDuration(3.0).sleep();

    ROS_WARN("Gotoinit start at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");
    initPose(group_arm);
    ROS_WARN("Gotoinit end at:%.8f",ros::Time::now().toSec());
    ROS_WARN("--------------------");

    ROS_WARN("round %d end at:%.8f",i,ros::Time::now().toSec());
    ROS_WARN("--------------------");
  
}

  private:

  //group_arm.setNumPlanningAttempts(2);
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Timer timer;
};




/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{

  ros::init(argc, argv, "own_pick_place_V4");
  pick_place node;
  node.main_loop();
  ros::AsyncSpinner spinner(1); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  

  return 0;
}