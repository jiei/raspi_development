#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

#ifndef pi
#define pi 3.14159265358979
#endif

//#define RELATIVE
#define ABSOLUTE

geometry_msgs::Vector3 velocity;
geometry_msgs::Pose2D pose;

void VelocityCallBack(const geometry_msgs::Vector3 &msg) {
  velocity = msg;
  ROS_INFO_STREAM_ONCE("receive!!");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mimic_localization");
  ros::NodeHandle n;
  ros::NodeHandle local_nh("~");

  ros::Subscriber velocity_sub =
      n.subscribe("target_velocity", 1, VelocityCallBack);
  ros::Publisher mimic_pose_pub =
      n.advertise<geometry_msgs::Pose2D>("mimic_pose", 1);
  ros::Rate loop_rate(30);

  double now = ros::Time::now().toSec();
  double old = now;
  double V = 0, phi = 0;

  velocity.x = 0;
  velocity.y = 0;
  velocity.z = 0;
  pose.x = 0;
  pose.y = 0;
  pose.theta = 0;

  while (ros::ok()) {
    now = ros::Time::now().toSec();

#ifdef RELATIVE
    if (velocity.z == 0.0) {
      V = sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
      if ((velocity.x == 0) && (velocity.y == 0))
        phi = 0;
      else
        phi = atan2(velocity.y, velocity.x);
      if (pose.x > -5.0 && pose.x < 5) {
        pose.x += V * cos(pose.theta + phi) * (now - old);
      }
      if (pose.y > -5.0 && pose.y < 5) {
        pose.y += V * sin(pose.theta + phi) * (now - old);
      }
    }
    pose.theta += velocity.z * (now - old);
    if (pose.theta >= pi)
      pose.theta = pose.theta - 2 * pi;
    else if (pose.theta <= (-1) * pi)
      pose.theta = 2 * pi - pose.theta;
#endif

#ifdef ABSOLUTE
    if ((pose.x > -2.0 && pose.x < 2) || (pose.x < -2.0 && velocity.x > 0) ||
        (pose.x > 2.0 && velocity.x < 0)) {
      pose.x += velocity.x * (now - old);
    }
    if (pose.y > -2.0 && pose.y < 2 || (pose.y < -2.0 && velocity.y > 0) ||
        (pose.y > 2.0 && velocity.y < 0)) {
      pose.y += velocity.y * (now - old);
    }
    pose.theta += velocity.z * (now - old);
    if (pose.theta >= pi)
      pose.theta = pose.theta - 2 * pi;
    else if (pose.theta <= (-1) * pi)
      pose.theta = 2 * pi - pose.theta;
#endif

    old = now;

    mimic_pose_pub.publish(pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
