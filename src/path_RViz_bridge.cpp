#include <nav_msgs/Path.h>
#include <raspi_development/path_to_RViz.h>
#include <ros/ros.h>

bool PathCallBack(raspi_development::path_to_RViz::Request &req,
                  raspi_development::path_to_RViz::Response &res);

nav_msgs::Path path;
bool received = false;

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_RViz_bridge");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::ServiceServer path_server =
      n.advertiseService("path_bridge_srv", PathCallBack);
  ros::Publisher path_pub =
      n.advertise<nav_msgs::Path>("path_visualization", 1000);

  while (ros::ok()) {
    if (received) {
      path_pub.publish(path);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

bool PathCallBack(raspi_development::path_to_RViz::Request &req,
                  raspi_development::path_to_RViz::Response &res) {
  path = req.path_msg;
  res.call_back_msg = true;
  received = true;
  ROS_INFO("path service succeed !!!");
  return true;
}
