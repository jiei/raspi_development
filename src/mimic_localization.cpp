#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>

geometry_msgs::Vector3 velocity;
geometry_msgs::Pose2D pose;

void VelocityCallBack(const geometry_msgs::Vector3& msg){
  velocity=msg;
  ROS_INFO_STREAM_ONCE("receive!!");
}

int main(int argc,char **argv){
  ros::init(argc,argv,"mimic_localization");
	ros::NodeHandle n;
 	ros::NodeHandle local_nh("~");

  ros::Subscriber velocity_sub=n.subscribe("target_velocity",1,VelocityCallBack);
 	ros::Publisher mimic_pose_pub=n.advertise<geometry_msgs::Pose2D>("mimic_pose",1);
 	ros::Rate loop_rate(30);

  double now=ros::Time::now().toSec();
  double old=now;

  velocity.x=0;
  velocity.y=0;
  velocity.z=0;
  pose.x=0;
  pose.y=0;
  pose.theta=0;

  while(ros::ok()){
    now=ros::Time::now().toSec();
    pose.x+=velocity.x*(now-old);
    pose.y+=velocity.y*(now-old);
    pose.theta+=velocity.z*(now-old);
    old=now;

    mimic_pose_pub.publish(pose);
    ros::spinOnce();
 		loop_rate.sleep();
  }
}
