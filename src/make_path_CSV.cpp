#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>

#define POINTS_INTERVAL 0.001 //[m]
#define BEZIER_PARAM_INTERVAL 0.0001

tf::Quaternion YawToQuaternion(double yaw);
bool endflag=false;

//座標のデータタイプ
struct Point2D {
  double x;
  double y;
};

//経路のデータタイプ
struct PathData {
  unsigned short N;             //何番目の経路集合か
  unsigned short n;             //その経路集合の中で何番目の経路か
  double max_velocity;          //最高速
  double final_velocity;        //最終点での目標速度
  double target_angle;          //目標機体角度
  std::vector<Point2D> points;  //経路座標の動的配列
};

void InputHeader(PathData *p);
void CreatePathPoints(PathData *p);
void WritePathOnFile(PathData p);
void WritePathOnRviz(PathData p);
void ChooseNextStep(PathData *p);
double CalcDistBetweenPoints(Point2D p1,Point2D p2);

PathData path={1,1,0,0,0,std::vector<Point2D>{1,Point2D{0,0}}};

int main(int argc,char **argv){
  ros::init(argc,argv,"make_path_CSV");
	ros::NodeHandle n;
 	ros::NodeHandle local_nh("~");

  if(argv[1]==NULL){
    ROS_ERROR("Invalid argment : Must input path-file name");
    return -1;
  }
  std::string fname="/home/jack/catkin_ws/src/raspi_development/src/path/"+std::string(argv[1]);
  std::ofstream fout(fname.c_str());
  if(!fout){
    ROS_ERROR("Cannot open %s",fname.c_str());
    return -1;
  }
  else ROS_INFO("opened %s",fname.c_str());

  while(ros::ok()){
    std::cout << "Editing path #" << path.N << "-" << path.n << '\n';
    InputHeader(&path);
    CreatePathPoints(&path);
    /*WritePathOnFile(path);
    WritePathOnRviz(path);*/
    ChooseNextStep(&path);
    if(endflag) break;
  }

  fout.close();

  return 0;
}

tf::Quaternion YawToQuaternion(double yaw){
  tf::Quaternion q;
  q.setRPY(0.0,0.0,yaw);
  return q;
}

void InputHeader(PathData *p){
  std::cout << "---------- Header making phase ----------" << '\n';
  std::cout << "First, input Max Velocity [m/s]: ";
  std::cin >> p->max_velocity;
  std::cout << "Second, input Final Velocity [m/s]: ";
  std::cin >> p->final_velocity;
  std::cout << "Third, input Target Angle [rad]: ";
  std::cin >> p->target_angle;
  std::cout << '\n';
}

void CreatePathPoints(PathData *p){
  std::cout << "--------- Path-points making phase----------" << '\n';

  //終点・制御点の入力
  Point2D start_point=p->points.back();
  Point2D end_point,control_point;
  char control_point_flag;
  p->points.clear();
  std::cout << "Start point is (" << start_point.x << "," << start_point.y << ") \n";
  std::cout << "Input end point x : ";
  std::cin >> end_point.x;
  std::cout << "                 y : ";
  std::cin >> end_point.y;
  std::cout << "You can add control points of Bezier Curve." << '\n';
  std::cout << "If you don't add control points, it will be straight line connecting start point and last point." << '\n';
  std::cout << "Do you add control points? (y/n) : ";
  std::cin >> control_point_flag;
  if(control_point_flag=='y') {
    std::cout << "Input control point x : ";
    std::cin >> control_point.x;
    std::cout << "                    y : ";
    std::cin >> control_point.y;
  }
  else{
    control_point.x=0;
    control_point.y=0;
  }
  std::cout << "Making Bezier curve ..." << '\n';
  std::cout << "   Start point is (" << start_point.x << "," << start_point.y << ") \n";
  std::cout << "   End point is (" << end_point.x << "," << end_point.y << ") \n";
  std::cout << "   Control point is (" << control_point.x << "," << control_point.y << ") \n";

  p->points.push_back(start_point);
  Point2D point_tmp;
  for(double t=0; t<=1; t+=BEZIER_PARAM_INTERVAL){
    point_tmp.x=(1-t)*(1-t)*start_point.x + 2*(1-t)*t*control_point.x + t*t*end_point.x;
    point_tmp.y=(1-t)*(1-t)*start_point.y + 2*(1-t)*t*control_point.y + t*t*end_point.y;
    if(CalcDistBetweenPoints(point_tmp,p->points.end())>POINTS_INTERVAL){
      p->points.push_back(point_tmp);
    }
  }
}

void WritePathOnFile(PathData p){

}

void WritePathOnRviz(PathData p){

}

void ChooseNextStep(PathData *p){
  char command;
  bool command_flag=false;
  std::cout << "----------Select next step----------" << '\n';
  do{
    std::cout << "n : add path on a same path assembly" << '\n';
    std::cout << "N : make new path assembly" << '\n';
    std::cout << "q : quit this program" << '\n';
    std::cout << "Input command: ";
    std::cin >> command;
    command_flag=false;
    switch(command){
      case 'n':
        p->n++;
        break;
      case 'N':
        p->N++;
        p->n=1;
        break;
      case 'q':
        endflag=true;
        break;
      default:
        command_flag=true;
        break;
    }
  }while(command_flag);

}

double CalcDistBetweenPoints(Point2D p1,Point2D p2){
  return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y);
}
