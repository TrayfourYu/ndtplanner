#ifndef GLOBALPLAN_H
#define GLOBALPLAN_H
#include "common_definition.h"
#include "map2D.h"
#include<queue>
#include<unordered_map>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tinyspline_ros/tinysplinecpp.h>
//#include "Vec3.h"
using namespace std;
using namespace daysun;

struct cmp
{
    bool operator()(SimpleSlope &a, SimpleSlope &b) const{
        return a.cost > b.cost;
    }
};


class AstarPlanar{
    priority_queue<SimpleSlope,vector<SimpleSlope>,cmp> open_queue;
    //list<Slope *> closed_list;
    unordered_map<string, unordered_map<int, unordered_map<int,SimpleSlope>>> visited;
    list<Slope *> global_path;
    list<SimpleSlope *> path_pose;
    // Searched path
    nav_msgs::Path path_;
    // Dense path
    nav_msgs::Path dense_path_;
    Vector3 start,goal;
    int angle_size;
    float curve_weight;
    float h_weight;
    float getup_weight;
    int primitive_num;
    int rip_num;
    float step_width;
    float step_min;
    float isRampAngle;
    double goal_radius;
    double descretized_angle;
    std::vector<std::vector<NodeUpdate>> state_update_table_;

    void createStateUpdateTable(){
        state_update_table_.resize(angle_size);
        for (int k = 0; k < angle_size; k++)
        { 
            state_update_table_[k].resize(primitive_num+rip_num);
        }

        for (int k = 0; k < angle_size; k++)
        {
            double robot_angle = descretized_angle * k;
            double step = step_width;
            // ROS_WARN_STREAM("!!!!!!!!!!!!!!!!!!!!!!!step is"<<step);
            // Calculate x and y shift to next state
            NodeUpdate nu;
            // forward
            nu.shift_x     = step * std::cos(robot_angle);
            nu.shift_y     = step * std::sin(robot_angle);
            nu.rotation    = 0;
            nu.index_theta = 0;
            nu.step        = step;
            nu.back        = false;
            state_update_table_[k][0] = nu;

            for (int l = 1; l < (primitive_num-1)/2+1; l++) {
                double turning_radius =step / (descretized_angle*l);

                // Calculate right and left circle
                // Robot moves along these circles
                double right_circle_center_x = turning_radius * std::sin(robot_angle);
                double right_circle_center_y = turning_radius * std::cos(robot_angle) * -1.0;
                double left_circle_center_x  = right_circle_center_x * -1.0;
                double left_circle_center_y  = right_circle_center_y * -1.0;

                // forward right
                nu.shift_x     = right_circle_center_x + turning_radius * std::cos(M_PI_2 + robot_angle - l*descretized_angle);
                nu.shift_y     = right_circle_center_y + turning_radius * std::sin(M_PI_2 + robot_angle - l*descretized_angle);
                nu.rotation    = descretized_angle * - l;
                nu.index_theta = -l;
                nu.step        = step;
                nu.curve       = l/primitive_num;
                nu.back        = false;
                state_update_table_[k][2*l-1] = nu;

                // forward left
                nu.shift_x     = left_circle_center_x + turning_radius * std::cos(-1.0 * M_PI_2 + robot_angle + l*descretized_angle);
                nu.shift_y     = left_circle_center_y + turning_radius * std::sin(-1.0 * M_PI_2 + robot_angle + l*descretized_angle);
                nu.rotation    = descretized_angle * l;
                nu.index_theta = l;
                nu.step        = step;
                nu.curve       = l/primitive_num;
                nu.back        = false;
                state_update_table_[k][2*l] = nu;

            }

            nu.shift_x     = step_min * std::cos(robot_angle);
            nu.shift_y     = step_min * std::sin(robot_angle);
            nu.rotation    = 0;
            nu.index_theta = 0;
            nu.step        = step_min;
            nu.back        = false;
            state_update_table_[k][primitive_num] = nu;

            for(int i=1; i<(rip_num-1)/2+1;i++){

                double turning_radius = step_min / (descretized_angle*i);

                // Calculate right and left circle
                // Robot moves along these circles
                double right_circle_center_x = turning_radius * std::sin(robot_angle);
                double right_circle_center_y = turning_radius * std::cos(robot_angle) * -1.0;
                double left_circle_center_x  = right_circle_center_x * -1.0;
                double left_circle_center_y  = right_circle_center_y * -1.0;

                // forward right
                nu.shift_x     = right_circle_center_x + turning_radius * std::cos(M_PI_2 + robot_angle - i*descretized_angle);
                nu.shift_y     = right_circle_center_y + turning_radius * std::sin(M_PI_2 + robot_angle - i*descretized_angle);
                nu.rotation    = descretized_angle * - i;
                nu.index_theta = -i;
                nu.step        = step_min;
                nu.curve       = i/rip_num;
                nu.back        = false;
                state_update_table_[k][primitive_num+2*i-1] = nu;

                // forward left
                nu.shift_x     = left_circle_center_x + turning_radius * std::cos(-1.0 * M_PI_2 + robot_angle + i*descretized_angle);
                nu.shift_y     = left_circle_center_y + turning_radius * std::sin(-1.0 * M_PI_2 + robot_angle + i*descretized_angle);
                nu.rotation    = descretized_angle * i;
                nu.index_theta = i;
                nu.step        = step_min;
                nu.curve       = i/rip_num;
                nu.back        = false;
                state_update_table_[k][primitive_num+2*i] = nu;
            }

        }
    }

    bool IsOnTheRamp(const Vector3& p1, const Vector3& p2, float angle){
        float delta_l = sqrt(pow(p1(0) - p2(0),2)+pow(p1(1) - p2(1),2));
        float delta_tan_theta = abs(p1(2) - p2(2)) / delta_l;
        if(delta_tan_theta >= tanf(M_PI / 180.0 *angle)) return true;
        else return false;
    }

public:
    AstarPlanar(Vector3 start,Vector3 goal, Config planner_config):start(start),goal(goal){
        angle_size = planner_config.angle_size;
        curve_weight = planner_config.curve_weight;
        h_weight = planner_config.h_weight;
        getup_weight = planner_config.getup_weight;
        primitive_num = planner_config.primitive_num;
        rip_num = planner_config.rip_num;
        isRampAngle = planner_config.is_ramp_angle;
        step_width = planner_config.step_width;
        descretized_angle = 2.0 * M_PI / angle_size;
        goal_radius = planner_config.goal_radius;
        step_min = planner_config.step_min;
        createStateUpdateTable();
    }

    bool findRoute(TwoDmap & map2D,RobotSphere & robot,string demand){
        double time_start3 = stopwatch();
        //find where the start is
        string morton_xy;
        int morton_z;
        float start_z;
        bool route  = false;
        map2D.transMortonXY(start,morton_xy);
        if(!map2D.getZneast(morton_xy, start(2), start_z)){
            cout<<"cant find start nearst z slope"<<endl;
            return false;
        }
        start(2) = start_z;
        map2D.transMortonZ(start_z, morton_z);
        map<string,Cell *>::iterator it = map2D.map_cell.find(morton_xy);
        float gridLen = map2D.getGridLen(); //resolution
        float zLen = map2D.getZLen(); //z-resolution

        if(it != map2D.map_cell.end()){
            map<int,Slope *,CmpByKeyUD>::iterator ss = (it->second)->map_slope.find(morton_z);
            if(ss != (it->second)->map_slope.end()){
                //起始点
                // ss->second->g = 0; //start
                // ss->second->f = ss->second->g + ss->second->h;
                // ss->second->status = STATUS::CLOSED;

                SimpleSlope start_slope(start, 0, ss->second->f, 0, STATUS::OPEN);
                start_slope.morton_xy = morton_xy;
                start_slope.morton_z = morton_z;
                open_queue.emplace(start_slope);
                visited[morton_xy][morton_z][0] = start_slope;
                //开始搜索
                //cout<<">>>>>Begin Search<<<<<"<<endl;
                int search_count = 0;
                while(!open_queue.empty()){
                    cout<<"-------Search Count Is "<<++search_count<<"-------"<<endl;
                    SimpleSlope tmp = open_queue.top();
                    open_queue.pop();
                    string cur_morton_xy = tmp.morton_xy;
                    int cur_morton_z = tmp.morton_z;

                    Slope * cur_slope = map2D.findSlope(cur_morton_xy, cur_morton_z);

                    float cur_x = tmp.pos(0), cur_y = tmp.pos(1), cur_z = tmp.pos(2);
                    int cur_theta;
                    //map2D.transXYMorton(cur_x, cur_y, cur_morton_xy);
                    cur_theta = tmp.index_theta;
                    visited[cur_morton_xy][cur_morton_z][cur_theta].status = STATUS::CLOSED;
                    SimpleSlope * cur = &visited[cur_morton_xy][cur_morton_z][cur_theta];

                    //if find the goal
                    if((abs(cur_x - goal(0)) <= goal_radius)
                        && (abs(cur_y - goal(1)) <= goal_radius)
                        && (abs(cur_z - goal(2)) <= goal_radius)){
                        route = true;
//                        cout<<"find end "<<temp->morton_xy<<","<<temp->morton_z<<endl;
                        Slope * goal_slope = map2D.findSlope(cur_morton_xy, cur_morton_z);
                        global_path.push_front(goal_slope);
                        path_pose.push_front(cur);
                        cout<<"found the route to goal\n";
                        break;
                    }
                    //find neighbors
                    float cmd=2.5;
                    if(demand.compare("true") == 0){
                        cmd=4;
                    }

                    bool isRamp = false;
                    if(cur->parent != NULL){
                        if(IsOnTheRamp(cur->parent->pos, cur->pos, isRampAngle))
                            isRamp = true;
                    }
                   
                    // for each update
                    for(const auto &state : state_update_table_[cur_theta]){
                        if(isRamp && abs(state.rotation) >= descretized_angle) continue;
                        // Next state
                        float next_x     = cur_x + state.shift_x;
                        float next_y     = cur_y + state.shift_y;
                        int next_theta = cur_theta + state.index_theta;
                        // Avoid invalid index
                        next_theta = (next_theta + angle_size) % angle_size;
                        float steer_cost  = state.step * state.curve;

                        string next_morton_xy;
                        Vector3 next_pos(next_x, next_y, 0);
                        map2D.transMortonXY(next_pos, next_morton_xy);

                        //cout<<"x: "<<next_x<<", y: "<<next_y<<endl;

                        list<Slope *> neiSlope;
                        map2D.getReachable(next_morton_xy,neiSlope,robot,cur_morton_z,cur_slope->normal,cur_slope->mean,cmd);
                        list<Slope *>::iterator itS = neiSlope.begin();

    //                    //for each traversible neighbors
                         while(itS != neiSlope.end()){
                            if((*itS)->h == FLT_MAX) {
                                itS++;
                                continue;
                            }
                            int next_morton_z = (*itS)->morton_z;
                            SimpleSlope *next_simple_slope = &visited[next_morton_xy][next_morton_z][next_theta];
                            if(next_simple_slope->status == STATUS::OPEN /*|| next_simple_slope->status == STATUS::CLOSED*/){
                                ///test
                                Vector3 q,itn;
                                ///use the mean
                                q = cur_slope->mean;
                                itn = (*itS)->mean;
                                float getup_cost = getup_weight * sqrt(pow(q(2)-itn(2), 2));
                                if((next_simple_slope->gc) > (cur->gc + getup_cost + steer_cost)){
                                    //remove and insert
                                    //update g and f, and father node
//                                    cout<<"itS g"<<(*itS)->g<<endl; ///dont know if its right
//                                    cout<<"itemp g "<<(itTemp->second)->g<<endl;
                                    next_simple_slope->morton_xy = next_morton_xy;
                                    next_simple_slope->morton_z = next_morton_z;
                                    next_simple_slope->index_theta = next_theta;
                                    next_pos(2) = map2D.countPositionZ(next_morton_z);
                                    next_simple_slope->pos = next_pos;
                                    next_simple_slope->gc = cur->gc + getup_cost + steer_cost;
                                    next_simple_slope->cost = next_simple_slope->gc + h_weight * (*itS)->h;
                                    next_simple_slope->status = STATUS::OPEN;
                                    next_simple_slope->parent = cur;
                                    open_queue.emplace(*next_simple_slope);

                                    // (*itS)->g = next_simple_slope->cost;
                                    // (*itS)->f = (*itS)->g + (*itS)->h;
                                    (*itS)->father = cur_slope;

                                    //cout<<"z: "<<next_morton_z<<" is OPEN"<<endl;
                                }
                            }
                            else if(next_simple_slope->status == STATUS::NONE){
                                //update g and f, and father node
                                //insert into OPEN
                                ///test
                                Vector3 q,itn;
                                q = cur_slope->mean;
                                itn = (*itS)->mean;
                                float getup_cost = getup_weight * sqrt(pow(q(2)-itn(2), 2));
                                // (*itS)->status = STATUS::OPEN;
                                // (*itS)->g = cur_slope->g + map2D.TravelCost(/*temp->mean,(*itS)->mean*/q,itn);
                                // (*itS)->f = (*itS)->g + (*itS)->h;

                                next_simple_slope->morton_xy = next_morton_xy;
                                next_simple_slope->morton_z = next_morton_z;
                                next_simple_slope->index_theta = next_theta;
                                next_pos(2) = map2D.countPositionZ(next_morton_z);
                                next_simple_slope->pos = next_pos;
                                next_simple_slope->gc = cur->gc + getup_cost + steer_cost;
                                next_simple_slope->cost = next_simple_slope->gc +  h_weight * (*itS)->h;
                                next_simple_slope->status = STATUS::OPEN;
                                next_simple_slope->parent = cur;
                                open_queue.emplace(*next_simple_slope);

                                (*itS)->father = cur_slope;

                                //cout<<"z: "<<next_morton_z<<" is OPEN"<<endl;
                                //cout<<"gc: "<<next_simple_slope->gc<<", hc: "<<(*itS)->h<<endl;
                            }
                            itS++;
                         }
                    }
                }            
            }else{
                cout<<"1,Sth wrong with the start slope, cant find it.\n";
                return false;
            }
        }else{
            cout<<"2,Sth wrong with the start slope, cant find it.\n";
            return false;
        }
        if(route){
            Slope * i = global_path.front();
            SimpleSlope * j = path_pose.front();
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "/my_frame";
            path_.header = header;
            while(i->father != NULL && j->parent!=NULL){
                global_path.push_front(i->father);
                path_pose.push_front(j->parent);

                Vector3f normal = map2D.GetGroundNormal(i, robot);

                tf::Quaternion rot(normal(0), normal(1), normal(2), 1.0);
                tf::Matrix3x3 m(rot);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                geometry_msgs::PoseStamped ros_pose;
                ros_pose.pose.position.x = (j->pos)(0);
                ros_pose.pose.position.y = (j->pos)(1);
                ros_pose.pose.position.z = (j->pos)(2);
                ros_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, j->index_theta * descretized_angle);
                ros_pose.header = header;
                path_.poses.push_back(ros_pose);

                i = global_path.front();
                j = path_pose.front();
            }
    
            Vector3f normal = map2D.GetGroundNormal(i, robot);

            tf::Quaternion rot(normal(0), normal(1), normal(2), 1.0);
            tf::Matrix3x3 m(rot);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Set path as ros message
            geometry_msgs::PoseStamped ros_pose;
            ros_pose.pose.position.x = (j->pos)(0);
            ros_pose.pose.position.y = (j->pos)(1);
            ros_pose.pose.position.z = (j->pos)(2);
            ros_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, j->index_theta * descretized_angle);
            ros_pose.header = header;
            path_.poses.push_back(ros_pose);

            std::reverse(path_.poses.begin(), path_.poses.end());
            double time_end3 = stopwatch();
            cout<<"Global planr done. A*: "<<(time_end3-time_start3)<<" s\n";
            return true;
        }else{
            cout<<"not find the road\n";
            return false;
        }
    }

    void samplePathByStepLength(double step,TwoDmap & map2D, RobotSphere & robot) {
        if (this->path_.poses.empty()) {
            return;
        }
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "/my_frame";
        this->dense_path_.header = header;
        this->dense_path_.poses.clear();
        geometry_msgs::PoseStamped ros_pose;
        ros_pose.header = header;
        for (std::size_t i = 0; i < path_.poses.size() - 1 ; i++) {
            double x1, y1, z1, x2, y2, z2;
            x1 = path_.poses.at(i).pose.position.x;
            y1 = path_.poses.at(i).pose.position.y;
            z1 = path_.poses.at(i).pose.position.z;
            x2 = path_.poses.at(i + 1).pose.position.x;
            y2 = path_.poses.at(i + 1).pose.position.y;
            z2 = path_.poses.at(i + 1).pose.position.z;

            double t1 = astar::modifyTheta(tf::getYaw(path_.poses.at(i).pose.orientation));
            double t2 = astar::modifyTheta(tf::getYaw(path_.poses.at(i + 1).pose.orientation));
            double delta_t = t2 - t1;

            if(fabs(delta_t) > M_PI)
                delta_t = (-2 * M_PI + fabs(delta_t)) * delta_t / fabs(delta_t);
                // -pi < delta_t < pia
            double delta_s = std::hypot(x2 - x1, y2 - y1);

            // straight line case
            if (delta_t == 0) {
                // calculate how many points need to be inserted by step
                //cout<<"----z1: "<<z1<<", z2: "<<z2<<endl;
                double num = delta_s / step;
                int size = static_cast<int>(num);
                for (int j = 0; j < size; j++) {
                    ros_pose.pose.position.x = x1 + cos(t1) * step * j;
                    ros_pose.pose.position.y = y1 + sin(t2) * step * j;
                    ros_pose.pose.position.z = z1 + (j * (z2 - z1) / (size - 1)) ;
                    InsertOrien(map2D, robot, ros_pose, t1);
                    dense_path_.poses.push_back(ros_pose);
                }
            } else if (delta_t > 0) {
                // left turn
                // arc length
                double R = delta_s / (sqrt(2*(1 - cos(delta_t)))); //余弦定理
                double l = R * delta_t;
                double center_x = x1 + cos(t1 + M_PI / 2.0)*R;
                double center_y = y1 + sin(t1 + M_PI / 2.0)*R;
                // delta arc length now becomes l
                double num = l / step;
                int size = static_cast<int>(num);
                for (int j = 0; j < size; j++) {
                    double heading = t1 + j*step / l * delta_t;
                    ros_pose.pose.position.x = center_x + cos(heading - M_PI / 2.0)*R;
                    ros_pose.pose.position.y = center_y + sin(heading - M_PI / 2.0)*R;
                    ros_pose.pose.position.z = z1 + (j * (z2 - z1) / (size - 1));
                    InsertOrien(map2D, robot, ros_pose, heading);
                    //ros_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
                    dense_path_.poses.push_back(ros_pose);
                }
            
            } else {
                // right turn
                // arc length
                double R = delta_s / (sqrt(2*(1 - cos(delta_t))));
                double l = R * fabs(delta_t);
                double center_x = x1 + cos(t1 - M_PI / 2.0)*R;
                double center_y = y1 + sin(t1 - M_PI / 2.0)*R;
                // delta arc length now becomes l
                double num = l / step;
                
                int size = static_cast<int>(num);
                for (int j = 0; j < size; j++) {
                    double heading = t1 + j*step / l * delta_t;
                    ros_pose.pose.position.x = center_x + cos(heading + M_PI / 2.0)*R;
                    ros_pose.pose.position.y = center_y + sin(heading + M_PI / 2.0)*R;
                    ros_pose.pose.position.z = z1 + (j * (z2 - z1) / (size - 1));
                    InsertOrien(map2D, robot, ros_pose, heading);
                    //ros_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
                    dense_path_.poses.push_back(ros_pose);
                }
            }
        }
        savingForSpeed(toStatePath(),map2D, robot);
    }

    void InsertOrien(TwoDmap & map2D, RobotSphere & robot, geometry_msgs::PoseStamped& ros_pose, const double& heading){
        Slope* tmp = map2D.GetSlopeFromXYZ(ros_pose.pose.position.x, ros_pose.pose.position.y,ros_pose.pose.position.z);
        if(tmp == NULL){
            tf::Quaternion rot(dense_path_.poses.back().pose.orientation.x, dense_path_.poses.back().pose.orientation.y,dense_path_.poses.back().pose.orientation.z, dense_path_.poses.back().pose.orientation.w);
            tf::Matrix3x3 m(rot);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            ros_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading);
        }else{
            Vector3f normal = map2D.GetGroundNormal(tmp, robot);
            tf::Quaternion rot(normal(0), normal(1), normal(2), 1.0);
            tf::Matrix3x3 m(rot);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            ros_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, heading);
        }
    }


    void showRoute(TwoDmap & map2D,ros::Publisher marker_pub){
        map2D.showSlopeList(marker_pub,global_path,4);
        cout<<"route show done\n";
    }

    void showDensePath(ros::Publisher& marker_pub){
        if(ros::ok()){
            if (marker_pub.getNumSubscribers() == 1){
                marker_pub.publish(dense_path_);
            }
        }
    }

    std::vector<State> toStatePath() {
        std::vector<State> path;
        State state;
        for (const auto &pose : dense_path_.poses) {
            state.pose = state.pose.msgToPose(pose.pose);
            // tf::Quaternion rot(pose.pose.orientation.x, pose.pose.orientation.y,
            //                    pose.pose.orientation.z,pose.pose.orientation.w);
            // tf::Vector3 normal = rot.getAxis();
            // normal.normalize();
            // cout<<"normal_x: "<<normal.x()<<", normal_y: "<<normal.y()<<", normal_z: "<<normal.z()<<endl;
            // state.n = Point3(normal.x(), normal.y(), normal.z());
            path.emplace_back(state);
        }
        return path;
    }

    void savingForSpeed(const std::vector<State> &input, TwoDmap & map2D, RobotSphere & robot) {
        std::vector<double> ctrlp;
        for (const auto &state : input) {
            ctrlp.push_back(state.pose.x);
            ctrlp.push_back(state.pose.y);
            ctrlp.push_back(state.pose.z);
        }
        tinyspline::BSpline curve(ctrlp.size() / 3, 3, 8, TS_CLAMPED);
        curve.setControlPoints(ctrlp);
        auto dcurve = curve.derive();
        auto ddcurve = dcurve.derive();
        size_t size = 1e4;
        double du = 1.0 / static_cast<double>(size);
        std::vector<State> path;
        State state;
        state.s = 0;
        int count = 0;
        for (size_t i(0); i <= size; ++i) {
            const double u = du * static_cast<double>(i);
            auto result = curve.eval(u).result();
            //position
            state.pose =
                Pose3d(result[0], result[1], result[2]);
            if (i == 0 || i == size ||
                path.back().pose.EuclidDis2D(state.pose) >= 0.1) {
                result = dcurve.eval(u).result();
                Point3 dr(result[0], result[1], result[2]);
                result = ddcurve.eval(u).result();
                Point3 ddr(result[0], result[1], result[2]);
                double vel = dr.norm();
                //alpha
                state.alpha = dr / vel;

                Mat<3> proj = state.alpha * state.alpha.transpose();
                proj = Mat<3>::Identity() - proj;
                auto kn = proj * ddr / vel / vel;
                //kappa
                state.k = kn.norm();  // dr.cross(ddr).norm() / pow(vel, 3);
                //beta
                state.beta = kn / state.k;
                //s
                if (i > 0) {
                    state.s = path.back().s + path.back().pose.EuclidDis2D(state.pose);
                } else {
                    state.s = 0;
                }

                //n
                Slope * tmp = map2D.findNearstSlope(state.pose.x, state.pose.y, state.pose.z, 
                                                    robot.getReachableHeight());
                if(tmp != NULL){
                    Vector3f normal = map2D.GetGroundNormal(tmp, robot);
                    state.n = Point3(normal(0),normal(1), normal(2));
                }else{
                    state.n = path.back().n;
                }
                //euler_angl
                Mat<3> rot_matrix;
                rot_matrix.col(0) = state.alpha;
                rot_matrix.col(2) = state.n;
                rot_matrix.col(1) = state.n.cross(state.alpha);
                Quater quat(rot_matrix);
                tf::Quaternion rot(quat.x(), quat.y(),quat.z(),quat.w());
                tf::Matrix3x3 m(rot);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                state.euler_angle = Vector<3>(yaw, pitch, roll);
                path.emplace_back(state);
                count++;
            }
        }
        printf(">>>> input %d pose, after fitting get %d points\n", (int)input.size(),
                (int)path.size());
        std::ofstream fout;
        fout.open("/home/mr_csc/yu_ws/cache/3dpath_for_speed_planner.csv", std::ios::out);
        fout << "x,y,z,tx,ty,tz,qx,qy,qz,nx,ny,nz,s,kappa,psi,theta,phi\n";
        for (const auto &state : path) {
            fout << std::setprecision(12) << state.pose.x << ","
                << std::setprecision(12) << state.pose.y << ","
                << std::setprecision(12) << state.pose.z << ","
                << std::setprecision(12) << state.alpha.x() << ","
                << std::setprecision(12) << state.alpha.y() << ","
                << std::setprecision(12) << state.alpha.z() << ","
                << std::setprecision(12) << state.beta.x() << ","
                << std::setprecision(12) << state.beta.y() << ","
                << std::setprecision(12) << state.beta.z() << ","
                << std::setprecision(12) << state.n.x() << "," << std::setprecision(12)
                << state.n.y() << "," << std::setprecision(12) << state.n.z() << ","
                << std::setprecision(12) << state.s << "," << std::setprecision(12)
                << state.k << "," << std::setprecision(12) << state.euler_angle(2)
                << "," << std::setprecision(12) << state.euler_angle(1) << ","
                << std::setprecision(12) << state.euler_angle(0) << "\n";
        }
        fout.close();
    }

};
#endif // GLOBALPLAN_H
