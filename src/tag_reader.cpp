#include "geometry_msgs/Point.h"
#include <ros/ros.h>
#include "robo/tag.h"

geometry_msgs::Point tags;

void tag_callback(const geometry_msgs::Point tag_read)
{
  tags = tag_read;
  ROS_INFO("tags received!", tags);
}

bool read_tags(robo::tag::Request &req, robo::tag::Response &res );

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_reader");
  ros::NodeHandle n;

  ros::Subscriber tag_sub = n.subscribe("tags", 1, tag_callback);

  ros::ServiceServer nav = n.advertiseService("tag_read", read_tags);
  ROS_INFO("server ready to be read tags!");

  ros::spin();
  return 0;
}

bool read_tags(robo::tag::Request &req, robo::tag::Response &res )
{
  res.tag_result = tags;
  return true;
}