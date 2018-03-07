#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <termios.h>
// mode
#define LOGICOOL
//#define PS3
//#define KEY

/*
    button data -> Int8 button
      1 : Logicool
      2 : start
      3 : A
      4 : B
      5 : X
      6 : Y
*/

std_msgs::Int8 button;
geometry_msgs::Vector3 velocity;

void getKey();

double Base_Linear_Vel = 0, Base_Angular_Vel = 0;

void JoyCallBack(const sensor_msgs::Joy &msg) {
#ifdef LOGICOOL
  velocity.x = (-1) * Base_Linear_Vel * msg.axes[0];
  velocity.y = Base_Linear_Vel * msg.axes[1];
  if (msg.buttons[4])
    velocity.z = Base_Angular_Vel;
  else if (msg.buttons[5])
    velocity.z = (-1) * Base_Angular_Vel;
  else
    velocity.z = 0;

  button.data = 0;
  button.data += msg.buttons[8];      // Logicool
  button.data += msg.buttons[7] << 1; // start
  button.data += msg.buttons[0] << 2; // A
  button.data += msg.buttons[1] << 3; // B
  button.data += msg.buttons[2] << 4; // X
  button.data += msg.buttons[3] << 5; // Y
#endif

#ifdef PS3
  velocity.x = (-1) * Base_Linear_Vel * msg.axes[0];
  velocity.y = Base_Linear_Vel * msg.axes[1];
  if (msg.buttons[10])
    velocity.z = Base_Angular_Vel;
  else if (msg.buttons[11])
    velocity.z = (-1) * Base_Angular_Vel;
  else
    velocity.z = 0;
#endif

  ROS_INFO_STREAM_ONCE("HAVE JOY !!");
  // ROS_INFO("x=%f y=%f theta=%f",velocity.x,velocity.y,velocity.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_to_velocity");
  ros::NodeHandle n;
  ros::NodeHandle local_nh("~");

  ros::Subscriber joy_sub = n.subscribe("/joy", 1, JoyCallBack);
  ros::Publisher velocity_pub =
      n.advertise<geometry_msgs::Vector3>("/target_velocity", 1);
  ros::Publisher button_pub = n.advertise<std_msgs::Int8>("/button_state", 1);
  ros::Rate loop_rate(30);

  if (!local_nh.hasParam("Base_Linear_Vel")) {
    ROS_INFO("Parameter Base_Linear_Vel is not defined. Now, it is set default "
             "value");
    local_nh.setParam("Base_Linear_Vel", 0.3);
  }
  if (!local_nh.getParam("Base_Linear_Vel", Base_Linear_Vel)) {
    ROS_ERROR("No value set on Base_Linear_Vel");
    return -1;
  }

  if (!local_nh.hasParam("Base_Angular_Vel")) {
    ROS_INFO("Parameter Base_Angular_Vel is not defined. Now, it is set "
             "default value");
    local_nh.setParam("Base_Angular_Vel", 0.3);
  }
  if (!local_nh.getParam("Base_Angular_Vel", Base_Angular_Vel)) {
    ROS_ERROR("No value set on Base_Angular_Vel");
    return -1;
  }

  while (ros::ok()) {
    velocity_pub.publish(velocity);
    button_pub.publish(button);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
