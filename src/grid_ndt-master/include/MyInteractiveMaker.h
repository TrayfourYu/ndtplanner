#ifndef MYINTERACTIVEMAKER_HPP
#define MYINTERACTIVEMAKER_HPP
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <fstream>
#include <map>
#include "common_definition.h"
typedef visualization_msgs::InteractiveMarker InteractiveMarker;
typedef visualization_msgs::InteractiveMarkerControl InteractiveMarkerControl;

class MyInteractiveMaker
{
private:
    bool ready_ = false;
    std::map<std::string, geometry_msgs::Pose> cached_pose_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    std::map<std::string, Pose3d> pose_list_;
    void feedbackProcess(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void makeButtonMarker(const tf::Vector3 &position);
    void insertMarker(const std::string &name);
    void applyChange() { this->server_->applyChanges(); }
    Pose3d msgToPose(const geometry_msgs::Pose &msg);
public:
    MyInteractiveMaker(/* args */);
    ~MyInteractiveMaker()=default;
    void initInteractiveMarker(const std::string &topic);
    const Pose3d startPose()const {
        auto it = this->pose_list_.find("start_pose");
        if (it == pose_list_.end()) {
            assert(false);
        }
        return it->second;
    };
    const Pose3d goalPose() const {
        auto it = this->pose_list_.find("goal_pose");
        if (it == pose_list_.end()) {
            assert(false);
        }
        return it->second;
    };
    bool IsReady(){return this->ready_;};
};

MyInteractiveMaker::MyInteractiveMaker(/* args */)
{
    std::ifstream fin;
    fin.open("/home/mr_csc/yu_ws/cache/startandgoal.csv");
    if (fin.is_open()) {
        double tx, ty, tz, qx, qy, qz, qw;
        std::string name;
        while (fin >> name >> tx >> ty >> tz >> qw >> qx >> qy >> qz) {
            cached_pose_[name].position.x = tx;
            cached_pose_[name].position.y = ty;
            cached_pose_[name].position.z = tz;
            cached_pose_[name].orientation.w = qw;
            cached_pose_[name].orientation.x = qx;
            cached_pose_[name].orientation.y = qy;
            cached_pose_[name].orientation.z = qz;
        }
        printf(">>>>Debug: read %lu cached pose\n", cached_pose_.size());
    }
}

void MyInteractiveMaker::initInteractiveMarker(const std::string &topic) {
  server_.reset(
      new interactive_markers::InteractiveMarkerServer(topic, "", false));
  makeButtonMarker(tf::Vector3(10, 10, -1));
  insertMarker("start_pose");
  insertMarker("goal_pose");
  applyChange();
  printf("Set interactive marker topic");
}

void MyInteractiveMaker::feedbackProcess(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  //if (!feedback->mouse_point_valid) {
    //printf("%s pose is invalid!\n", feedback->marker_name.c_str());
  //}
  //printf("%s updated!\n", feedback->marker_name.c_str());

  if (feedback->marker_name == "button") {
    std::ofstream fout;
    fout.open("/home/mr_csc/yu_ws/cache/startandgoal.csv", std::ios::out);
    if (fout.is_open()) {
      for (const auto &it : pose_list_) {
        fout << it.first << " " << it.second.x << " "
             << it.second.y << " " << it.second.z << it.second.qw << " " << it.second.qx
             << " " << it.second.qy << " " << it.second.qz << "\n";
      }
    }
    fout.close();
    this->ready_ = true;
    return;
  }

  Pose3d pose = msgToPose(feedback->pose);
  auto it = pose_list_.find(feedback->marker_name);
  if (it != pose_list_.end()) {
    it->second = pose;
  }
  applyChange();
}

void MyInteractiveMaker::makeButtonMarker(const tf::Vector3 &position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/my_frame";
  tf::pointTFToMsg(position, int_marker.pose.position);
  //    int_marker.scale = 1;
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl button_control;

  button_control.interaction_mode = InteractiveMarkerControl::BUTTON;
  button_control.name = "button_control";

  visualization_msgs::Marker shape_marker;
  shape_marker.type = visualization_msgs::Marker::CYLINDER;
  shape_marker.scale.x = 2;
  shape_marker.scale.y = 2;
  shape_marker.scale.z = 0.1;
  shape_marker.color.r = 0.5;
  shape_marker.color.g = 0.0;
  shape_marker.color.b = 0.0;
  shape_marker.color.a = 0.5;
  button_control.markers.push_back(shape_marker);
  button_control.always_visible = true;
  int_marker.controls.push_back(button_control);
  // create a interactive control which moving in 6 DOF
  visualization_msgs::InteractiveMarkerControl control;
  control.name = "move_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker,
                  boost::bind(&MyInteractiveMaker::feedbackProcess, this, _1));
}

void MyInteractiveMaker::insertMarker(const std::string &name) {
  geometry_msgs::Pose cache;
  auto it = this->cached_pose_.find(name);
  if (it != cached_pose_.end()) {
    cache = it->second;
  } else {
    cache.position.x = 0.0;
    cache.position.y = 0.0;
    cache.position.z = 0.0;
    cache.orientation.w = 1;
    cache.orientation.x = 0;
    cache.orientation.y = 0;
    cache.orientation.z = 0;
  }
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/my_frame";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = name;
  int_marker.description = name;
  int_marker.scale = 1.0;
  int_marker.pose = cache;
  // create a box marker
  visualization_msgs::Marker shape_marker;
  shape_marker.id = name.length();
  shape_marker.type = visualization_msgs::Marker::CUBE;
  shape_marker.scale.x = 1.0;
  shape_marker.scale.y = 0.7;
  shape_marker.scale.z = 0.5;
  shape_marker.color.r = 0.0;
  shape_marker.color.g = 0.0;
  shape_marker.color.b = 0.5;
  shape_marker.color.a = 0.7;
  shape_marker.header = int_marker.header;
  shape_marker.pose = int_marker.pose;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl static_control;
  static_control.always_visible = true;
  static_control.markers.push_back(shape_marker);
  static_control.interaction_mode = InteractiveMarkerControl::FIXED;
  // add the control to the interactive marker
  int_marker.controls.push_back(static_control);

  // create a interactive control which moving in 6 DOF
  visualization_msgs::InteractiveMarkerControl control;
  tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "rotate_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "rotate_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "rotate_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker,
                  boost::bind(&MyInteractiveMaker::feedbackProcess, this, _1));

  pose_list_[name] = msgToPose(cache);
}

Pose3d MyInteractiveMaker::msgToPose(const geometry_msgs::Pose &msg) {
  return Pose3d(msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.x, msg.orientation.y,msg.orientation.z,msg.orientation.w);
}
#endif
