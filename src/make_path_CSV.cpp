#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <raspi_development/path_to_RViz.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#define DEBUG
#define POINTS_INTERVAL 0.001 //[m]
#define BEZIER_PARAM_INTERVAL 0.0001

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

void InputHeader(PathData *p);
void CreatePathPoints(PathData *p);
void WritePathOnRViz(PathData p);
void ChooseNextStep(PathData *p);
double CalcDistBetweenPoints(Point2D p1, Point2D p2);

PathData path = {1, 1, 0, 0, 0, std::vector<Point2D>(1, Point2D{0, 0})};
nav_msgs::Path visualize_path, visualize_path_old;
bool save_path = true, endflag = false;

int main(int argc, char **argv) {
  ros::init(argc, argv, "make_path_CSV");
  ros::NodeHandle n;
  ros::NodeHandle local_nh("~");
  ros::ServiceClient path_client =
      n.serviceClient<raspi_development::path_to_RViz>("path_bridge_srv");

  raspi_development::path_to_RViz srv;

  if (argv[1] == NULL) {
    ROS_ERROR("Invalid argment : Must input path-file name");
    return -1;
  }
  std::string fname = "/home/jack/catkin_ws/src/raspi_development/src/path/" +
                      std::string(argv[1]);
  std::ofstream fout(fname.c_str());
  if (!fout) {
    ROS_ERROR("Cannot open %s", fname.c_str());
    return -1;
  } else {
    ROS_INFO("opened %s", fname.c_str());
  }

  while (ros::ok()) {
    std::cout << "Editing path #" << path.N << "-" << path.n << '\n';
    InputHeader(&path);
    CreatePathPoints(&path);
    WritePathOnRViz(path);

    srv.request.path_msg = visualize_path;
    if (path_client.call(srv)) {
      ROS_INFO("service succeed !!!");
    } else {
      ROS_ERROR("Fail to call path service");
    }
    sleep(1);

    ChooseNextStep(&path);
    if (save_path) {
      std::cout << "\n----------Path saving phase----------" << '\n';
      std::cout << "Saving path data on " << std::string(argv[1]) << " ..."
                << '\n';

      fout << path.N << ',' << path.n << ',' << path.max_velocity << ','
           << path.final_velocity << ',' << path.target_angle << std::endl;
      for (int i = 0; i < path.points.size(); i++) {
        fout << path.points[i].x << '/' << path.points[i].y << ',';
      }
      fout << '.' << std::endl; //ピリオドがパスデータ終了の合図
    }
    if (endflag) {
      break;
    }
  }
  fout << '.' << std::endl;
  fout.close();

  return 0;
}

void InputHeader(PathData *p) {
  std::cout << "---------- Header making phase ----------" << '\n';
  std::cout << "First, input Max Velocity [m/s]: ";
  std::cin >> p->max_velocity;
  std::cout << "Second, input Final Velocity [m/s]: ";
  std::cin >> p->final_velocity;
  std::cout << "Third, input Target Angle [rad]: ";
  std::cin >> p->target_angle;
  std::cout << '\n';
}

void CreatePathPoints(PathData *p) {
  std::cout << "--------- Path-points making phase----------" << '\n';

  //終点・制御点の入力
  Point2D start_point = p->points.back();
  Point2D end_point, control_point;
  char control_point_flag;
  p->points.clear();
  std::cout << "Start point is (" << start_point.x << "," << start_point.y
            << ") \n";
  std::cout << "Input end point x : ";
  std::cin >> end_point.x;
  std::cout << "                y : ";
  std::cin >> end_point.y;
  std::cout << "You can add control points of Bezier Curve." << '\n';
  std::cout << "If you don't add control points, it will be a straight line "
               "connecting start point and last point."
            << '\n';
  std::cout << "Do you add control points? (y/n) : ";
  std::cin >> control_point_flag;
  if (control_point_flag == 'y') {
    std::cout << "Input control point x : ";
    std::cin >> control_point.x;
    std::cout << "                    y : ";
    std::cin >> control_point.y;
  } else {
    control_point.x = start_point.x;
    control_point.y = start_point.y;
  }
  std::cout << "Making Bezier curve ..." << '\n';
  std::cout << "   Start point is (" << start_point.x << "," << start_point.y
            << ") \n";
  std::cout << "   End point is (" << end_point.x << "," << end_point.y
            << ") \n";
  std::cout << "   Control point is (" << control_point.x << ","
            << control_point.y << ") \n";

  p->points.push_back(start_point);
  Point2D point_tmp;
  for (double t = 0; t <= 1; t += BEZIER_PARAM_INTERVAL) {
    point_tmp.x = (1 - t) * (1 - t) * start_point.x +
                  2 * (1 - t) * t * control_point.x + t * t * end_point.x;
    if (point_tmp.x > -0.0001 && point_tmp.x < 0.0001) {
      point_tmp.x = 0;
    }
    point_tmp.y = (1 - t) * (1 - t) * start_point.y +
                  2 * (1 - t) * t * control_point.y + t * t * end_point.y;
    if (point_tmp.y > -0.0001 && point_tmp.y < 0.0001) {
      point_tmp.y = 0;
    }
    if (CalcDistBetweenPoints(point_tmp, p->points.back()) >
        (POINTS_INTERVAL * POINTS_INTERVAL)) {
      p->points.push_back(point_tmp);
    }
  }
  p->points.push_back(end_point);
  std::cout << "Created " << p->points.size() << " points." << '\n';
}

void WritePathOnRViz(PathData p) {
  std::cout << "\n--------- Path visualization phase----------" << '\n';

  geometry_msgs::PoseStamped geo_pose;
  tf::Quaternion q;

  visualize_path.header.frame_id = "path_frame";
  q.setRPY(0.0, 0.0, p.target_angle);

  for (int i = 0; i < p.points.size(); i++) {
    geo_pose.header.frame_id = "path_frame";
    geo_pose.pose.position.x = p.points[i].x;
    geo_pose.pose.position.y = p.points[i].y;
    geo_pose.pose.position.z = 0;
    geo_pose.pose.orientation.x = q.x();
    geo_pose.pose.orientation.y = q.y();
    geo_pose.pose.orientation.z = q.z();
    geo_pose.pose.orientation.w = q.w();

    visualize_path.poses.push_back(geo_pose);
  }
}

void ChooseNextStep(PathData *p) {
  char command;
  bool command_flag = false;
  std::cout << "\n----------Select next step----------" << '\n';
  do {
    std::cout << "n : add path on a same path assembly" << '\n';
    std::cout << "N : make new path assembly" << '\n';
    std::cout << "r : cancel and recreate a path on the same number" << '\n';
    std::cout << "q : quit this program" << '\n';
    std::cout << "Input command: ";
    std::cin >> command;
    command_flag = false;
    switch (command) {
    case 'n':
      p->n++;
      break;
    case 'N':
      p->N++;
      p->n = 1;
      break;
    case 'r':
      save_path = false;

      // p->pointsを始点だけにする
      Point2D tmp;
      tmp = p->points.front();
      p->points.clear();
      p->points.push_back(tmp);
      // visualize_pathを一つ前の状態にする
      visualize_path = visualize_path_old;

      break;
    case 'q':
      endflag = true;
      break;
    default:
      command_flag = true;
      break;
    }
  } while (command_flag);

  visualize_path_old = visualize_path;
}

double CalcDistBetweenPoints(Point2D p1, Point2D p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}
