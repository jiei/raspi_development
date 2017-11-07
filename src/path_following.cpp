#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <raspi_development/path_to_RViz.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <tf/transform_datatypes.h>
#include <typeinfo>

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

void InputPath(PathData *path, std::string file_name);

std::vector<PathData> path;

int main(int argc, char **argv) {}

void InputPath(PathData *path, std::string file_name) {
  std::ifstream ifs(file_name);
  if (!ifs) {
    std::cout << "Error : Cannnot open the file" << '\n';
  } else {

    while (!ifs.eof()) {
      PathData path_tmp;
      std::string header_buf, path_buf;
      ifs >> header_buf;
      ifs >> path_buf;
      std::cout << header_buf << '\n';
      std::cout << path_buf << '\n';
      std::istringstream streambuffer(header_buf);
      std::string token;
      for (int i = 0; !std::getline(streambuffer, token, ','); i++) {
        switch (i) {
        case 0:
          path_tmp.N = (unsigned short)std::stoi(token);
          break;
        case 1:
          path_tmp.n = (unsigned short)std::stoi(token);
          break;
        case 2:
          path_tmp.max_velocity = std::stof(token);
          break;
        case 3:
          path_tmp.final_velocity = std::stof(token);
          break;
        case 4:
          path_tmp.target_angle = std::stof(token);
          break;
        }
      }
    }
  }
}
