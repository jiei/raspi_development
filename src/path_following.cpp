#include <cstdlib>
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <raspi_development/path_to_RViz.h>
#include <ros/ros.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tf/transform_datatypes.h>
#include <typeinfo>

using namespace std;

//座標のデータタイプ
struct Point2D {
  double x;
  double y;
};

//経路のデータタイプ
struct PathData {
  unsigned short N;      //何番目の経路集合か
  unsigned short n;      //その経路集合の中で何番目の経路か
  double max_velocity;   //最高速
  double final_velocity; //最終点での目標速度
  double target_angle;   //目標機体角度
  std::vector<Point2D> points; //経路座標の動的配列
};

std::vector<PathData> path;
geometry_msgs::Vector3 velocity;
geometry_msgs::Pose2D machine_pose;

void PoseCallBack(const geometry_msgs::Pose2D &msg) {
  machine_pose.x = msg.x;
  machine_pose.y = msg.y;
  machine_pose.theta = msg.theta;
  ROS_INFO_STREAM_ONCE("Received location !!");
}

void InputPath(std::vector<PathData> *path, std::string file_name);

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_following");
  ros::NodeHandle n;
  ros::NodeHandle local_nh("~");
  ros::ServiceClient path_client =
      n.serviceClient<raspi_development::path_to_RViz>("path_bridge_srv");
  ros::Publisher velocity_pub =
      n.advertise<geometry_msgs::Vector3>("/target_velocity", 1);
  ros::Subscriber loc_sub = n.subscribe("/machine_pose", 1, PoseCallBack);

  ros::Rate loop_rate(30);

  if (argv[1] == NULL) {
    ROS_ERROR("Invalid argment : Must input path-file name");
    return -1;
  }
  std::string fname = "/home/jack/catkin_ws/src/raspi_development/src/path/" +
                      std::string(argv[1]);

  InputPath(&path, fname);
  for (int i = 0; i < path.size(); i++) {
    std::cout << "N = " << path[i].N << '\n';
    std::cout << "n = " << path[i].n << '\n';
    std::cout << "max_velocity = " << path[i].max_velocity << '\n';
    std::cout << "final_velocity = " << path[i].final_velocity << '\n';
    std::cout << "target_angle = " << path[i].target_angle << '\n';
    std::cout << "first_point = (" << path[i].points[0].x << ","
              << path[i].points[0].y << ")" << '\n';
    std::cout << "point_num = " << path[i].points.size() << '\n';
  }

  while (ros::ok()) {
    ros::spinOnce();
    PathFollowing();
    velocity_pub.publish(velocity);
    loop_rate.sleep();
  }

  return 0;
}

void InputPath(std::vector<PathData> *path, std::string file_name) {
  int line_count = 0;
  PathData path_tmp;
  std::ifstream ifs;
  ifs.open(file_name);
  if (!ifs) {
    std::cout << "Error : Cannnot open the file" << '\n';
  }

  while (!ifs.eof()) {

    std::string token;
    if (line_count % 2 == 0) { // headerの入力
      std::string header_buf;
      std::getline(ifs, header_buf);
      // std::cout << header_buf << '\n';
      if (*header_buf.c_str() == '.') {
        // std::cout << "!" << '\n';
        break;
      }
      std::istringstream sb_h(header_buf);
      for (int i = 0; i < 5; i++) {
        std::getline(sb_h, token, ',');
        // std::cout << token << '\n';
        switch (i) {
        case 0:
          // path_tmp[line_count / 2].N = (unsigned short)stoi(token);
          path_tmp.N = (unsigned short)stoi(token);
          break;
        case 1:
          // path_tmp[line_count / 2].n = (unsigned short)stoi(token);
          path_tmp.n = (unsigned short)stoi(token);
          break;
        case 2:
          // path_tmp[line_count / 2].max_velocity = stof(token);
          path_tmp.max_velocity = stof(token);
          break;
        case 3:
          // path_tmp[line_count / 2].final_velocity = stof(token);
          path_tmp.final_velocity = stof(token);
          break;
        case 4:
          // path_tmp[line_count / 2].target_angle = stof(token);
          path_tmp.target_angle = stof(token);
          break;
        }
      }

    } else if (line_count % 2 == 1) { // pathの入力
      std::string path_buf;
      std::getline(ifs, path_buf);
      // std::cout << path_buf << '\n';
      std::istringstream sb_p(path_buf);
      string x_tmp, y_tmp;
      Point2D point_tmp;
      path_tmp.points.clear();
      for (int i = 0; std::getline(sb_p, token, ','); i++) {
        if (token == ".")
          break;
        // std::cout << token << '\n';
        std::istringstream sb_c(token);
        std::getline(sb_c, x_tmp, '/');
        std::getline(sb_c, y_tmp, '/');
        point_tmp.x = stod(x_tmp);
        point_tmp.y = stod(y_tmp);

        path_tmp.points.push_back(point_tmp);
      }

      /*std::cout << "N = " << path_tmp.N << '\n';
      std::cout << "n = " << path_tmp.n << '\n';
      std::cout << "max_velocity = " << path_tmp.max_velocity << '\n';
      std::cout << "final_velocity = " << path_tmp.final_velocity << '\n';
      std::cout << "target_angle = " << path_tmp.target_angle << '\n';
      std::cout << "first_point = (" << path_tmp.points[0].x << ","
                << path_tmp.points[0].y << ")" << '\n';
      std::cout << "second_point = (" << path_tmp.points[1].x << ","
                << path_tmp.points[1].y << ")" << '\n';
      std::cout << "point_num = " << path_tmp.points.size() << '\n';*/

      path->push_back(path_tmp);
    }
    line_count++;
  }
}

double CalcDistOfBezier(int Ps[2], int Pc[2], int Pf[2], int Pt[2],
                        double *p_t) {
  double P_sample[2][10];
  double sample_distance2[10];
  double min_distance[2];

  double t_interval = 0.1;
  double t_cand[2] = {0, 1};
  double P_min[2][2];
  double distance;
  double The_point[2];
  int count = 0;

  while (t_interval > 0.000001) {

    for (int i = 0; i < 10; i++) {
      P_sample[0][i] =
          BezierPlots(Ps[0], Pc[0], Pf[0], t_cand[0] + i * t_interval);
      P_sample[1][i] =
          BezierPlots(Ps[1], Pc[1], Pf[1], t_cand[0] + i * t_interval);
      sample_distance2[i] =
          (Pt[0] - P_sample[0][i]) * (Pt[0] - P_sample[0][i]) +
          (Pt[1] - P_sample[1][i]) * (Pt[1] - P_sample[1][i]);
      count++;
    }

    min_distance[0] = sample_distance2[0];
    min_distance[1] = 0;

    for (int i = 0; i < 10; i++) {
      if (sample_distance2[i] <= min_distance[0]) {
        min_distance[0] = sample_distance2[i];
        min_distance[1] = i;
      }
    }

    if (min_distance[1] == 0) {
      *p_t = t_cand[0];
      The_point[0] = BezierPlots(Ps[0], Pc[0], Pf[0], t_cand[0]);
      The_point[1] = BezierPlots(Ps[1], Pc[1], Pf[1], t_cand[0]);
      break;
    } else if (min_distance[1] == 9) {
      *p_t = t_cand[1];
      The_point[0] = BezierPlots(Ps[0], Pc[0], Pf[0], t_cand[1]);
      The_point[1] = BezierPlots(Ps[1], Pc[1], Pf[1], t_cand[1]);
      break;
    }

    t_cand[0] = t_cand[0] + (min_distance[1] - 1) * t_interval;
    t_cand[1] = t_cand[0] + (min_distance[1] + 1) * t_interval;

    t_interval = (t_cand[1] - t_cand[0]) / 20;

    P_min[0][0] = BezierPlots(Ps[0], Pc[0], Pf[0], t_cand[0]);
    P_min[0][1] = BezierPlots(Ps[1], Pc[1], Pf[1], t_cand[0]);
    P_min[1][0] = BezierPlots(Ps[0], Pc[0], Pf[0], t_cand[1]);
    P_min[1][1] = BezierPlots(Ps[1], Pc[1], Pf[1], t_cand[1]);

    distance = sqrt((P_min[0][0] - P_min[1][0]) * (P_min[0][0] - P_min[1][0]) +
                    (P_min[0][1] - P_min[1][1]) * (P_min[0][1] - P_min[1][1]));

    if (distance < 0.01) {
      *p_t = (t_cand[0] + t_cand[1]) / 2;
      The_point[0] =
          BezierPlots(Ps[0], Pc[0], Pf[0], (t_cand[0] + t_cand[1]) / 2);
      The_point[1] =
          BezierPlots(Ps[1], Pc[1], Pf[1], (t_cand[0] + t_cand[1]) / 2);
      break;
    }
  }

  printf("count : %d\n", count);
  return sqrt(pow(The_point[0] - Pt[0], 2.0) + pow(The_point[1] - Pt[1], 2.0));
}
