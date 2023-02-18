#ifndef GLOBALPLAN_H
#define GLOBALPLAN_H
#include "map2D.h"
#include<queue>
#include<unordered_map>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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

public:
    AstarPlanar(Vector3 start,Vector3 goal, Config planner_config):start(start),goal(goal){
        angle_size = planner_config.angle_size;
        curve_weight = planner_config.curve_weight;
        h_weight = planner_config.h_weight;
        getup_weight = planner_config.getup_weight;
        primitive_num = planner_config.primitive_num;
        rip_num = planner_config.rip_num;
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

                    double cur_x = tmp.pos(0), cur_y = tmp.pos(1), cur_z = tmp.pos(2);
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
                    Vector3f z_mormal(0,0,1);
                    if(map2D.countAngle(cur_slope->normal, z_mormal) >= 0.5 * robot.getAngle())
                        isRamp = true;
                    
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

                // Set tf pose
                tf::Vector3 origin((j->pos)(0), (j->pos)(1), (j->pos)(2));
                tf::Pose tf_pose;
                tf_pose.setOrigin(origin);
                tf_pose.setRotation(tf::createQuaternionFromYaw(j->index_theta * descretized_angle));

                // Transform path to global frame
                //tf_pose = map2ogm_ * tf_pose;

                // Set path as ros message
                geometry_msgs::PoseStamped ros_pose;
                tf::poseTFToMsg(tf_pose, ros_pose.pose);
                ros_pose.header = header;
                path_.poses.push_back(ros_pose);

                i = global_path.front();
                j = path_pose.front();
            }
            
            tf::Vector3 origin((j->pos)(0), (j->pos)(1), (j->pos)(2));
            tf::Pose tf_pose;
            tf_pose.setOrigin(origin);
            tf_pose.setRotation(tf::createQuaternionFromYaw(j->index_theta * descretized_angle));

            // Transform path to global frame
            //tf_pose = map2ogm_ * tf_pose;

            // Set path as ros message
            geometry_msgs::PoseStamped ros_pose;
            tf::poseTFToMsg(tf_pose, ros_pose.pose);
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

    void samplePathByStepLength(double step) {
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
                    ros_pose.pose.orientation = path_.poses.at(i).pose.orientation;
                    dense_path_.poses.push_back(ros_pose);

                    //cout<<"size: "<<size<<", j / size: "<<(j * (z2 - z1) / (size - 1))<<", interval z: "<<ros_pose.pose.position.z<<endl;
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
                    ros_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
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
                    ros_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
                    dense_path_.poses.push_back(ros_pose);
                }
            }
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

};
#endif // GLOBALPLAN_H
