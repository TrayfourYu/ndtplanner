#ifndef ROBOT_H
#define ROBOT_H
#include "Vec3.h"
#include "Vector3.h"
#include "Stopwatch.h"
#include "MyInteractiveMaker.h"
#define ROBOT_TAKEUP_CELL 1   //suppose the robot take up 1*1 cells
using namespace std;

class RobotSphere{
    float r; //radius
    Vector3 position;
    Vector3 goal;
    float reachableHeight;//height that it can reach
    float reachableRough;
    float reachableAngle;
    float start_yaw;
public:
//    list<Slope *> trajectory;
    //test
    //robot 0.25/0.5
    //gridLen 0.25
    //pos 2.75533,1.36108,1.67499
    //goal -0.52865,0.00212,1.66626
    //goal under the floor 0.02865,-0.50212,1.66626

    //fr2.pcd 0.5 0.5
    //pos  -3.75533,1.36108,-0.1499
    //goal 15.02865,1.1212,0.40626

    //fr2-16 0.5 0.5
    //30.02865,1.2212,0.40626
    //63.02865,-37.2212,1.3026
    RobotSphere()=default;
    RobotSphere(const float rr = 0.5, Vector3 pos= Vector3(30.02865,1.2212,0.40626),
                Vector3 goal=Vector3(63.02865,-37.2212,1.3026), float height = 0.15, 
                float rough = 100, float angle = 30):r(rr),position(pos),goal(goal), 
                reachableHeight(height),reachableRough(rough),reachableAngle(angle){
    }
    float getRobotR() {return r;}
//    float getR(){return r/ROBOT_TAKEUP_CELL;} //get cell radius
    Vector3 getPosition(){return position;}
    Vector3 getGoal(){return goal;}
    float getReachableHeight(){
        return reachableHeight;}
    float getRough(){
        //should be changed
        return reachableRough;
    }
    float getAngle(){
        return reachableAngle;
    }
    void setR(float ra){
        r = ra;
    }
    void setHeight(float h){
        reachableHeight = h;
    }
    void setRough(float ro){
        reachableRough = ro;
    }
    void setAngle(float an){
        reachableAngle = an;
    }
    void setPos(string s){
        vector<string> v;
        SplitString(s, v,",");
        position(0) = strToFloat(v[0]);
        position(1) = strToFloat(v[1]);
        position(2) = strToFloat(v[2]);
    }
    void setPos(Pose3d p){
        position(0) = p.x;
        position(1) = p.y;
        position(2) = p.z;
    }
    void setGoal(string s){
        vector<string> v;
        SplitString(s, v,",");
        goal(0) = strToFloat(v[0]);
        goal(1) = strToFloat(v[1]);
        goal(2) = strToFloat(v[2]);
    }
    void setGoal(Pose3d p){
        goal(0) = p.x;
        goal(1) = p.y;
        goal(2) = p.z;
    }
    void setYaw(float y){
        start_yaw = y;
    }
    float getYaw(){
        return start_yaw;
    }

    };


#endif // ROBOT_H
