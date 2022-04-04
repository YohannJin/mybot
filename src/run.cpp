#include "ros/ros.h"
#include "robo/action.h"
#include <queue>
#include <iostream>
#include <string>
#include <cstdlib>
#include <ros/duration.h>

#include "geometry_msgs/Point.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Path.h"
#include "robo/tag.h"


using namespace std;
bool grasp_done = false;
bool nav_done = false;



void grasp_done_callback(const std_msgs::Int16::ConstPtr &msg)
{
  if(msg->data == 1) grasp_done = true;
  else if(msg->data == 0) grasp_done = false;
  return;
}


// void teb_planner_callback(const nav_msgs::Path &msg)
// {
//   double old_xy = msg.poses[0].pose.position.x*msg.poses[0].pose.position.x + msg.poses[0].pose.position.y*msg.poses[0].pose.position.y;
//   double new_xy = msg.poses[1].pose.position.x*msg.poses[1].pose.position.x + msg.poses[1].pose.position.y*msg.poses[1].pose.position.y;
//   if (abs(new_xy - old_xy) < 10.0) 
//   {
//     nav_done = true;
//   }
//   else nav_done = false;
// }

bool nav_to_goal(string);
bool grasp(string);
bool place(string);
bool wait(double);

ros::ServiceClient nav_client;
ros::ServiceClient grasp_client;
ros::ServiceClient place_client;
ros::ServiceClient tag_client;
//ros::Subscriber gripper_sub;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "decision");
  ros::NodeHandle n;
  nav_client = n.serviceClient<robo::action>("navigation");
  grasp_client = n.serviceClient<robo::action>("grasp");
  place_client = n.serviceClient<robo::action>("place");
  tag_client = n.serviceClient<robo::tag>("tag_read");


  //gripper_sub = n.subscribe("grasp_done", 1, grasp_done_callback);

  // ros::Subscriber teb_planer_sub = n.subscribe("move_base/TebLocalPlannerROS/local_plan", 10, teb_planner_callback);

  float tag[] = {9.0, 9.0, 9.0};
  
  enum state{st, ck, s1, s2, s3};
  enum comb{c1, c2, c3, c4, c5, c6, c7, c8, c9, c10};

  //c1 []

  //check tags and store in tag[]
  nav_to_goal("check");
  ROS_INFO("going to check");
  wait(4);

  geometry_msgs::Point tags;
  robo::tag srv;
  srv.request.ask_tag = true;

  if(tag_client.call(srv))
  {
    ROS_INFO("asking tags");
    tags = srv.response.tag_result;
    
  }
  
  tag[0] = tags.x;
  tag[1] = tags.y;
  tag[2] = tags.z;
  // ROS_INFO("get ", tag[0],tag[1],tag[2]);

  // //recheck if tag not fully read
  // if (tag[0] == 9.0 || tag[1] == 9.0 || tag[2] == 9.0 )
  // {
  //   ROS_INFO("tags not fully detected, check agian!");
  //   nav_to_goal("check");
  //   wait(4);
  //   tag[0] = tags.x;
  //   tag[1] = tags.y;
  //   tag[2] = tags.z;
  // }

  //from check
  string cube_in_hand = "1";

  if(tag[0] == 0.0)
  {
    nav_to_goal("1a");
    wait(4);
    grasp("1");
    nav_to_goal("s1");
    wait(7);
    place("1");
  }
  else if(tag[0] == 1.0)
  {
    nav_to_goal("2");
    wait(3);
    grasp("2");
    nav_to_goal("s1");
    wait(5);
    place("1");
  }
  else if(tag[0] == 2.0)
  {
    nav_to_goal("3");
    wait(5);
    grasp("3");
    nav_to_goal("s1");
    wait(5);
    place("1");
  }
  else if(tag[0] == 3.0)
  {
    nav_to_goal("4");
    wait(9);
    grasp("1");
    nav_to_goal("s1");
    wait(9);
    place("1");
  }
  else if(tag[0] == 4.0)
  {
    nav_to_goal("5");
    wait(9);
    grasp("1");
    nav_to_goal("s1");
    wait(7.5);
    place("1");
  }
  else
  {
    ROS_INFO("1st nav failed: tag invalid!");
  }



  ROS_INFO("second cube!");
  //from station
  for(int i = 1; i < 3; i++)
  {
    if(tag[i] == 0)
    {
      nav_to_goal("1b");
      wait(8);
      grasp("1");
      if(i == 1)
      {
        nav_to_goal("s2");
        wait(8);
        place("2");
      }
      else if(i == 2)
      {
        nav_to_goal("s3");
        wait(7);
        place("3");
      }
    }
    else if(tag[i] == 1)
    {
      nav_to_goal("2");
      wait(5);
      grasp("2");
      if(i == 1)
      {
        nav_to_goal("s2");
        wait(5);
        place("2");
      }
      else if(i == 2)
      {
        nav_to_goal("s3");
        wait(5);
        place("3");
      }
    }
    else if(tag[i] == 2)
    {
      nav_to_goal("3");
      wait(4);
      grasp("3");
      if(i == 1)
      {
        nav_to_goal("s2");
        wait(4.5);
        place("2");
      }
      else if(i == 2)
      {
        nav_to_goal("s3");
        wait(4.5);
        place("3");
      }
    }
    else if(tag[i] == 3)
    {
      nav_to_goal("4");
      wait(8);
      grasp("1");
      if(i == 1)
      {
        nav_to_goal("s2");
        wait(8);
        place("2");
      }
      else if(i == 2)
      {
        nav_to_goal("s3");
        wait(10);
        place("3");
      }
    }
    else if(tag[i] == 4)
    {
      nav_to_goal("5");
      wait(8);
      grasp("1");
      if(i == 1)
      {
        nav_to_goal("s2");
        wait(8);
        place("2");
      }
      else if(i == 2)
      {
        nav_to_goal("s3");
        wait(8);
        place("3");
      }
    }
    else
    {
      ROS_INFO("nav failed: tag invalid!");
    }
    ROS_INFO("third cube!");
  }

  ros::spin();
  return 0;
}

bool nav_to_goal(string goal)
{
  robo::action srv;
  srv.request.nav_goal = goal;
  if (nav_client.call(srv))
  {
    ROS_INFO("start navigation!");
    return true;
  }
  else return false;
}

bool grasp(string mode)
{
  robo::action srv;
  srv.request.grasp = mode;
  if (grasp_client.call(srv))
  {
    if(mode == "2")
    {
      ros::Duration(0.1).sleep();
    }
    else if(mode == "3")
    {
      ros::Duration(0.1).sleep();
    }
    else ros::Duration(5).sleep();
    ROS_INFO("timeout! let's go!");
    // if(grasp_done) ROS_INFO("grasp finished");
    // else ros::Duration(2).sleep();
    srv.request.grasp = "0";
    grasp_client.call(srv);
    //cube_in_hand = true;
    return true;
  }
  else return false;
}

bool place(string sink)
{

  robo::action srv;
  srv.request.place = sink;
  if (place_client.call(srv))
  {
    ros::Duration(18).sleep();
    ROS_INFO("timeout! let's go!");
    // if(grasp_done) ROS_INFO("grasp finished");
    // else ros::Duration(2).sleep();
    srv.request.place = "0";
    place_client.call(srv);

    //cube_in_hand = false;
    return true;
  }
  else return false;
}

bool wait(double time)
{
  ros::Duration(time).sleep();
  robo::action srv;
  srv.request.nav_goal = "cancel";
  if (nav_client.call(srv))
  {
    ROS_INFO("old goal canceled, let's continue!");
    return true;
  }
  else return false;
}


