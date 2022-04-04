#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/duration.h>
#include <string>
#include "std_msgs/String.h"
#include "robo/action.h"

using namespace std;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool nav_to_goal(robo::action::Request &req, robo::action::Response &res );

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_goals");
  ros::NodeHandle n;
  // //publisher
  // ros::Publisher require_action_pub = n.advertise<int>("require_action", 1);
  // ros::Publisher action_reset_pub = n.advertise<int>("action_state", 1);
  // //subscribers
  // ros::Subscriber action_sub = n.subscribe<int>("action_state", 1, action_state);
  
  //server response
  ros::ServiceServer nav = n.advertiseService("navigation", nav_to_goal);
  ROS_INFO("server ready to accept navigation target!");
  ros::spin();
  return 0;
}

bool nav_to_goal(robo::action::Request &req, robo::action::Response &res)
{
  bool cancel = false;
  string target = req.nav_goal;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //x, y postion and z, w orientation
  double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
  bool goal_reached = false;

  if(target == "1a")
  {
    x = 0.14;
    y = 2.81;
    z =  0.7;
    w = 0.7;
    
  }
  else if(target == "1b")
  {
    x = 0.4;
    y = 3.2;
    z = 1.0;
    w = 0.0;
  }
  else if(target == "2")
  {
    x = -0.025;
    y = 2.3139124546051025;
    z = 0;
    w = 1;
    // z = -0.6936474426639455;
    // w = 0.72;
  }
  else if(target == "3")
  {
    x = 2.18;
    y = 2.71;
    z = 0.0;
    w = 1.0;
    //a bit steering
  }
  else if (target == "4")
  {
    x = 2.062978744506836;
    y = 0.23;
    z = -1.0;
    w = 0.0;
    //fixed
  }
  else if (target == "5")
  {
    x = 2.82;
    y = -0.92;
    z = 0.012344430517360736;
    w = 0.9999238046147326;
    //fixd
  }
  else if(target == "s1")
  {
    // x = 1.2478;
    // y = 1.9;
    // z = 0.0;
    // w = 1.0;
    x = 1.1;
    y = 1.7380278015136719;
    z = 0.0;
    w = 1.0;
  }
  else if(target == "s2")
  {
    x = 1.1;
    y = 1.7380278015136719;
    z = 0.0;
    w = 1.0;
  }
  else if(target == "s3")
  {
    // x = 1.26;
    // y = 1.58;
    // z = 0.0;
    // w = 1.0;
    x = 1.1;
    y = 1.7380278015136719;
    z = 0.0;
    w = 1.0;
  }
  else if(target == "check")
  {
    x = 0.31816983222961426;
    y = 1.5692417621612549;
    z = 0.05841524259722063;
    w = 0.9982923717189809;
  }
  else if(target == "place")
  {
    x = 1.1;
    y = 1.7;
    z = 0.0;
    w = 1.0;
  }
  else if(target == "cancel")
  {
    cancel = true;
  }
  else
  {
      ROS_INFO("not a valid target!");
      return false;
  }
  
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.z =  z;
  goal.target_pose.pose.orientation.w = w;
  ROS_INFO("setting goal to target");
  
  if(cancel)
  {
    ROS_INFO("canelling goals..");
    ac.cancelAllGoals();
  }
  else ac.sendGoal(goal);


  // ros::Duration(7.5).sleep();
  res.finish = true;
  return true;
}



