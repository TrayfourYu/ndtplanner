#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <assert.h>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include "GlobalPlan.h"
#include <vector>
#include <fstream>
#include"omp.h"
#include "MyInteractiveMaker.h"
using namespace Eigen;
using namespace std;
using namespace daysun;
using namespace octomath;
typedef multimap<string,daysun::OcNode *>  MAP_INT_MORTON_MULTI;
typedef multimap<string,daysun::OcNode *>::iterator iterIntNode;

RobotSphere robot(0.5); //radius--variable--according to the range of map
//test-0.25, sys-0.125 vision-1.5 bag-1
daysun::TwoDmap map2D(0.5,0.1);
MyInteractiveMaker StartAndGoal{};
ros::Publisher marker_pub,change_pub,markerArray_pub,markerArray_pub2,marker_pub_bo,route_pub,dense_path_pub/*,del_pub*/;
string demand;
Config planner_config;

//ofstream outfile("/home/daysun/testPointsSys.txt", ofstream::app);

void uniformDivision( const pcl::PointXYZ temp,bool change){    
    string morton_xy;
    int morton_z;
    Vector3 temp3(temp.x,temp.y,temp.z);
//    cout<<"ok\n";
    map2D.transMortonXYZ(temp3,morton_xy,morton_z);
    //for change-record which xy have been changed
    if(change){
        if(map2D.changeMorton_list.size() != 0){
            list<string>::iterator it = find(map2D.changeMorton_list.begin(), map2D.changeMorton_list.end(), morton_xy);
            if (it == map2D.changeMorton_list.end()){ //not find
                map2D.changeMorton_list.push_back(morton_xy);
            }
        }else{
            map2D.changeMorton_list.push_back(morton_xy);
        }
//        cout<<"change morton: "<<morton_xy<<","<<morton_z<<endl;
    }

    if(map2D.map_xy.count(morton_xy) == 0){ //not found
        //add new node
        daysun::OcNode * node = new daysun::OcNode();
//        node->lPoints.push_back(temp);
//        node->pCloud_ptr->points.push_back (temp);
        node->test_cloud.points.push_back(temp);
        node->morton = morton_xy;
        node->z = morton_z;
        map2D.map_xy.insert(MAP_INT_MORTON_MULTI::value_type(morton_xy,node));
//        map2D.map_z.insert(make_pair(morton_z,node));
        map2D.morton_list.push_back(morton_xy);       
    }else{//find        
        iterIntNode beg = map2D.map_xy.lower_bound(morton_xy);
          iterIntNode end = map2D.map_xy.upper_bound(morton_xy);
          bool found = false;
          while(beg != end){
              if( (beg->second)->z == morton_z ){//string z
//                  (beg->second)->lPoints.push_back(temp);
                  (beg->second)->test_cloud.points.push_back(temp);
                  found = true;
                  break;
              }
              ++beg;
          }
          if(!found){
              daysun::OcNode * node = new daysun::OcNode();
//              node->lPoints.push_back(temp);
              node->test_cloud.points.push_back (temp);
              node->morton = morton_xy;
              node->z = morton_z;
              map2D.map_xy.insert(MAP_INT_MORTON_MULTI::value_type(morton_xy,node));
          }
    }
}

bool loadCloud(std::string &filename,pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  std::cout << "Loading file " << filename.c_str() << std::endl;
  //read cloud
  if (pcl::io::loadPCDFile(filename, *cloud))
  {
    std::cout << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    return false;
  }
  //remove NaN Points
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;
  return true;
}

///initial-one time
void solve(const string& filename)
{
   string sss = "/home/mr_csc/yu_ws/src/grid_ndt-master/2.5d-ndt-data/"+filename;
   std::string cloud_path(sss);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

   if (!loadCloud(cloud_path,cloud))
     return;

    cout<<cloud->points.size()<<endl;
    map2D.setCloudFirst(Vector3(cloud->points[0].x,cloud->points[0].y,cloud->points[0].z));
    cout<<"set first end\n";

    double time_start = stopwatch();
    for (int i=1;i<cloud->points.size();i++)
    {
        uniformDivision(cloud->points[i],false);        
    }
     double time_end = stopwatch();
//    outfile.close();
    cout<<"division time: "<<(time_end-time_start)<<" s\n";
    cout<<"grid length "<<map2D.getGridLen()<<", morton size: "<<map2D.morton_list.size()<<endl;
    double time_start1 = stopwatch();
    map2D.create2DMap(demand);
    double time_end1 = stopwatch();
    cout<<"calculate time: "<<(time_end1-time_start1)<<" s\n";

   if(marker_pub.getNumSubscribers()){
        map2D.showInital(marker_pub,0);
//        map2D.showBottom(marker_pub_bo);
        cout<<"initial show done\n";
    }

    cout<<">>>>>Waiting for the start and goal..."<<endl;
    while(ros::ok() && !StartAndGoal.IsReady()){
        ros::spinOnce();
    }
    
    robot.setPos(StartAndGoal.startPose());
    robot.setGoal(StartAndGoal.goalPose());
    robot.setYaw(StartAndGoal.startPose().yaw_);
    cout<<">>>>>Get the start and goal."<<endl;

    Vector3 goal(robot.getGoal());
    map2D.computeCost(goal,robot,markerArray_pub,markerArray_pub2,demand); //compute the cost map

    AstarPlanar globalPlanr(robot.getPosition(),robot.getGoal(), planner_config);
    if(globalPlanr.findRoute(map2D,robot,demand) && route_pub.getNumSubscribers()){
        globalPlanr.showRoute(map2D,route_pub);
        if(dense_path_pub.getNumSubscribers()){
            globalPlanr.samplePathByStepLength(0.1,map2D,robot);
            globalPlanr.showDensePath(dense_path_pub);
        }
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fullSys_listener");
  ros::start();
  ros::NodeHandle n;
//  demand = argv[1];
  ros::param::get("~demand",demand); //which way to create map,for test
  string pos;
  string goal;
  float resolution,slope_interval,z_resolution,RobotRange,
        reachableHeight, reachableRough, reachableAngle;
  //ros::param::get("~pos",pos);
  //ros::param::get("~goal",goal);
  string filename;
  ros::param::get("~filename",filename);
  ros::param::get("~RobotRange",RobotRange);
  ros::param::get("~reachableHeight",reachableHeight);
  ros::param::get("~reachableRough",reachableRough);
  ros::param::get("~reachableAngle",reachableAngle);
  ros::param::get("~resolution",resolution);
  ros::param::get("~z_resolution",z_resolution);
  ros::param::get("~slope_interval",slope_interval);

  ros::param::get("~angle_size",planner_config.angle_size);
  ros::param::get("~curve_weight",planner_config.curve_weight);
  ros::param::get("~h_weight",planner_config.h_weight);
  ros::param::get("~getup_weight",planner_config.getup_weight);
  ros::param::get("~primitive_num",planner_config.primitive_num);
  ros::param::get("~rip_num",planner_config.rip_num);
  ros::param::get("~step_width",planner_config.step_width);
  ros::param::get("~step_min",planner_config.step_min);
  ros::param::get("~goal_radius",planner_config.goal_radius);
  ros::param::get("~is_ramp_angle",planner_config.is_ramp_angle);
  ros::param::get("~ramp_steer_num",planner_config.ramp_steer_num);
//   float tmp_step = planner_config.step_width / 5.0;
//   planner_config.step_min = std::max(tmp_step, resolution);

  //robot.setPos(pos);
  //robot.setGoal(goal);
  robot.setR(RobotRange);
  robot.setHeight(reachableHeight);
  robot.setRough(reachableRough);
  robot.setAngle(reachableAngle);
  map2D.setLen(resolution);
  map2D.setZLen(z_resolution);
  map2D.setInterval(slope_interval);

  StartAndGoal.initInteractiveMarker("start_and_goal");

  marker_pub = n.advertise<visualization_msgs::MarkerArray>("initial_marker_array", 1000);
  //startandgoal_pub = n.advertise<visualization_msgs::MarkerArray>("start_goal_marker", 1000);
  markerArray_pub = n.advertise<visualization_msgs::MarkerArray>("traversibility_marker_array", 1000);
  markerArray_pub2 = n.advertise<visualization_msgs::MarkerArray>("tra_check_marker_array", 1000);
  change_pub = n.advertise<visualization_msgs::MarkerArray>("change_marker_array", 1000);
//  del_pub = n.advertise<visualization_msgs::MarkerArray>("del_marker_array", 1000);
  marker_pub_bo = n.advertise<visualization_msgs::MarkerArray>("bottom_marker_array", 1000);
  route_pub= n.advertise<visualization_msgs::MarkerArray>("route_marker_array", 1000);
  dense_path_pub= n.advertise<nav_msgs::Path>("dense_path_pub", 1000);

  solve(filename);
  if(ros::ok()){
    ros::spin();
    ros::shutdown();
  }
  return 0;
}

