#define _CRT_NONSTDC_NO_WARNINGS

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8MultiArray.h>
#include <stdio.h>
#include <termios.h>
// mode
#define LOGICOOL
//#define PS3
//#define KEY

std_msgs::Int8MultiArray button;
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
  ros::Rate loop_rate(30);

  if (!local_nh.hasParam("Base_Linear_Vel")) {
    ROS_INFO("Parameter Base_Linear_Vel is not defined. Now, it is set default "
             "value");
    local_nh.setParam("Base_Linear_Vel", 1.0);
  }
  if (!local_nh.getParam("Base_Linear_Vel", Base_Linear_Vel)) {
    ROS_ERROR("No value set on Base_Linear_Vel");
    return -1;
  }

  if (!local_nh.hasParam("Base_Angular_Vel")) {
    ROS_INFO("Parameter Base_Angular_Vel is not defined. Now, it is set "
             "default value");
    local_nh.setParam("Base_Angular_Vel", 0.5);
  }
  if (!local_nh.getParam("Base_Angular_Vel", Base_Angular_Vel)) {
    ROS_ERROR("No value set on Base_Angular_Vel");
    return -1;
  }

  while (ros::ok()) {
#ifdef KEY
    getKey();
#endif
    velocity_pub.publish(velocity);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

#ifdef KEY
void getKey() {
  char key;

  ROS_INFO("auf");

  if (getchar()) {
    switch (getchar()) {
    case 'q':
      velocity.x = (-1) * Base_Linear_Vel;
      velocity.y = Base_Linear_Vel;
      velocity.z = 0;
      break;
    case 'w':
      velocity.x = 0;
      velocity.y = Base_Linear_Vel;
      velocity.z = 0;
      break;
    case 'e':
      velocity.x = Base_Linear_Vel;
      velocity.y = Base_Linear_Vel;
      velocity.z = 0;
      break;
    case 'a':
      velocity.x = (-1) * Base_Linear_Vel;
      velocity.y = 0;
      velocity.z = 0;
      break;
    case 'd':
      velocity.x = Base_Linear_Vel;
      velocity.y = 0;
      velocity.z = 0;
      break;
    case 'z':
      velocity.x = (-1) * Base_Linear_Vel;
      velocity.y = (-1) * Base_Linear_Vel;
      velocity.z = 0;
      break;
    case 'x':
      velocity.x = 0;
      velocity.y = (-1) * Base_Linear_Vel;
      velocity.z = 0;
      break;
    case 'c':
      velocity.x = Base_Linear_Vel;
      velocity.y = (-1) * Base_Linear_Vel;
      velocity.z = 0;
      break;
    case 'o':
      velocity.x = 0;
      velocity.y = 0;
      velocity.z = Base_Angular_Vel;
      break;
    case 'p':
      velocity.x = 0;
      velocity.y = 0;
      velocity.z = (-1) * Base_Angular_Vel;
      break;
    default:
      velocity.x = 0;
      velocity.y = 0;
      velocity.z = 0;
      break;
    }
  }
}
#endif
