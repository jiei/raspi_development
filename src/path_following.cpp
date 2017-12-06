#include <cstdlib>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
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

void InputPath(std::vector<PathData> *path, std::string file_name);

std::vector<PathData> path;

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_following");
  ros::NodeHandle n;
  ros::NodeHandle local_nh("~");
  ros::ServiceClient path_client =
      n.serviceClient<raspi_development::path_to_RViz>("path_bridge_srv");

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
