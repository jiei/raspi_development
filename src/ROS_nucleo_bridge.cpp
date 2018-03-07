#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>

/*
To nucleo
  0:  target_velocity/x[m/s] -> ros_data[0][cm/s]
  1:  target_velocity/y[m/s] -> ros_data[1][cm/s]
  2:  target_velocity/rotation[rad/s] -> ros_data[2][crad/s]
  3:  button_state/data -> ros_data[3]

From nucleo
  0:  nuc_data/data[0][mm] -> pose/x[m]
  1:  nuc_data/data[1][mm] -> pose/y[m]
  2:  nuc_data/data[2][mrad] -> pose/angle[rad]
*/

std_msgs::Int16MultiArray ros_data, nuc_data;
geometry_msgs::Pose2D machine_pose;

void VelocityCallBack(const geometry_msgs::Vector3 &msg) {
  ros_data.data[0] = int(msg.x * 100);
  ros_data.data[1] = int(msg.y * 100);
  ros_data.data[2] = int(msg.z * 100);
  ROS_INFO_STREAM_ONCE("receive velocity !!");
}

void ButtonStateCallBack(const std_msgs::Int8 &msg) {
  ros_data.data[3] = msg.data;
  ROS_INFO_STREAM_ONCE("receive button state !!");
}

void NucleoCallBack(const std_msgs::Int16MultiArray &msg) {
  machine_pose.x = double(msg.data[0]) / 1000;
  machine_pose.y = double(msg.data[1]) / 1000;
  machine_pose.theta = double(msg.data[2]) / 1000;
  ROS_INFO_STREAM_ONCE("receive nucleo data !!");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ROS_nucleo_bridge");
  ros::NodeHandle n;
  ros::NodeHandle local_nh("~");

  ros::Subscriber vel_sub =
      n.subscribe("/target_velocity", 1, VelocityCallBack);
  ros::Subscriber but_sub =
      n.subscribe("/button_state", 1, ButtonStateCallBack);
  /*
  ros::Subscriber nuc_sub = n.subscribe("/nucleo_data", 1, NucleoCallBack);*/

  ros::Publisher ros_pub =
      n.advertise<std_msgs::Int16MultiArray>("/ROS_data", 1);
  ros::Publisher loc_pub = n.advertise<geometry_msgs::Pose2D>("/pose", 1);

  ros::Rate loop_rate(30);

  // initialize
  ros_data.data.clear();
  ros_data.data.push_back(0);
  ros_data.data.push_back(0);
  ros_data.data.push_back(0);
  ros_data.data.push_back(0);
  /*machine_pose.x = 0;
  machine_pose.y = 0;
  machine_pose.theta = 0;*/

  while (ros::ok()) {
    ros_pub.publish(ros_data);
    // loc_pub.publish(machine_pose);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
