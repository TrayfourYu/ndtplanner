//
// Created by yqf on 2023/03/23.
//

#ifndef CONTACT_POINT_H
#define CONTACT_POINT_H
#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "common_definition.h"
#include "Vector3.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
using namespace std;
namespace bg = boost::geometry;
using point_xy = bg::model::d2::point_xy<double>;
using polygon = bg::model::polygon<point_xy>;

struct Plane {
    double A;
    double B;
    double C;
    double D;
    Plane(Vector3 p1, Vector3 p2, Vector3 p3) {
        A = (p2(1) - p1(1)) * (p3(2) - p1(2)) - (p3(1) - p1(1)) * (p2(2) - p1(2));
        B = (p2(2) - p1(2)) * (p3(0) - p1(0)) - (p3(2) - p1(2)) * (p2(0) - p1(0));
        C = (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p3(0) - p1(0)) * (p2(1) - p1(1));
        D = -A * p1(0) - B * p1(1) - C * p1(2);
    }

    double DistanceToPlane(const Vector3& p) {
        double distance = abs(A*p(0) + B*p(1) + C*p(2) + D) / sqrt(A*A + B*B + C*C);
        return distance;
    }

    Vector3 GetNormal(){
        Vector3 normal(A, B, C);
        if(normal.dot(Vector3(0,0,1.0)) < 0.0)
            normal = normal*(-1.0);
        return normal.normalize();
    }

    bool GetZ(double x, double y, double& z){
        try {
            z = -(D+A*x+B*y)/C;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Exception caught: " << e.what() << std::endl;
            return false;
        }
    }
};

class ContactPoint{
private:
    double length_, width_;
    double l_, b_, h_;
    vector<Vector3> ground_;
    pair<double, vector<Vector3>> contact_points_;
    Vector3 normal_;
    Vector3 center_pos_;
    Vector3 centroid_pos_;
    double NESM_;
    double buffer_ = 0.05;

    Vector3 GetCentroid(const vector<Vector3>& contact_point, const Vector3& normal){
        return (contact_point[3]-contact_point[0])*(l_/length_)
                +(contact_point[1]-contact_point[0])*(b_/width_)
                + normal*h_ + contact_point[0];
    }
    
    bool IsInTheProjection(const vector<Vector3>& contact_point, const Vector3& centroid, int i){
        polygon poly;
        // Add points to create a plane
        for(int j=0;j<4;j++){
            if(j==i) continue;
            // cout<<"contact point: ("<<contact_point[j](0)<<", "<<contact_point[j](1)<<", "<<contact_point[j](2)<<") \n";
            bg::append(poly.outer(), point_xy(contact_point[j](0), contact_point[j](1)));
        }
        for(int j=0;j<4;j++){
            if(j==i) continue;
            else{
                bg::append(poly.outer(), point_xy(contact_point[j](0), contact_point[j](1)));
                break;
            }
        }
        // 定义一个二维点
        point_xy pt(centroid(0), centroid(1));

        // 使用点-多边形关系算法判断点是否在多边形内
        return bg::within(pt, poly);
    }

    bool FindContactPoint1(const vector<Vector3>& rectangle_ground){
        for(int i=0;i<4;i++){
            int ignore = i;
            vector<Vector3> contact_point = rectangle_ground;
            contact_point[i] = contact_point[(i+1)%4] + contact_point[(i+3)%4] 
                               - contact_point[(i+2)%4];
            double height_differ;
            //2
            height_differ = (contact_point[1](2) + contact_point[2](2))*0.5 - ground_[2](2);
            if(height_differ < -buffer_) continue;
            //5
            height_differ = (contact_point[0](2) + contact_point[3](2))*0.5 - ground_[5](2);
            if(height_differ < -buffer_) continue;
            height_differ = contact_point[i](2) - rectangle_ground[i](2);
            if(height_differ < -buffer_) continue;            
            if(abs(height_differ) <= buffer_){
                ignore = -1;
            }
            Plane contact_plane(contact_point[(i+1)%4],contact_point[(i+2)%4],contact_point[(i+3)%4]);
            Vector3 normal = contact_plane.GetNormal();
            Vector3 centroid = GetCentroid(contact_point, normal);
            if(!IsInTheProjection(contact_point, centroid, ignore)){
                //cout<<"prjection check failed! \n";
                continue;
            } 
            if(contact_points_.first > centroid(2)){
                contact_points_.first = centroid(2);
                contact_points_.second.clear();
                for(int j=0;j<4;j++){
                    if(j==ignore) continue;
                    contact_points_.second.emplace_back(contact_point[j]);
                }
                normal_ = normal;
                center_pos_ = (contact_point[(i+1)%4] + contact_point[(i+3)%4]) * 0.5;
                centroid_pos_ = centroid;
            }
        }
        return !(contact_points_.first == DBL_MAX);
    }

    bool FindContactPoint2(const vector<Vector3>& rectangle_ground){
        for(int i=0;i<4;i++){
            int ignore = i;
            vector<Vector3> contact_point = rectangle_ground;
            contact_point[i] = contact_point[(i+1)%4] + contact_point[(i+3)%4] 
                               - contact_point[(i+2)%4];
            double height_differ;
            //3号点
            height_differ = (contact_point[2](2)*2.0 - contact_point[1](2)) - ground_[3](2);
            if(height_differ < -buffer_) continue;
            //4号点
            height_differ = (contact_point[3](2)*2.0 - contact_point[0](2)) - ground_[4](2);
            if(height_differ < -buffer_) continue;
            height_differ = contact_point[i](2) - rectangle_ground[i](2);
            if(height_differ < -buffer_) continue;            
            if(abs(height_differ) <= buffer_){
                ignore = -1;
            }
            vector<Vector3> vehicle_point = {contact_point[0], contact_point[1], 
                                             contact_point[2]*2.0 - contact_point[1],
                                             contact_point[3]*2.0 - contact_point[0]};
            Plane contact_plane(contact_point[(i+1)%4],contact_point[(i+2)%4],contact_point[(i+3)%4]);
            Vector3 normal = contact_plane.GetNormal();
            Vector3 centroid = GetCentroid(vehicle_point, normal);
            if(!IsInTheProjection(contact_point, centroid, ignore)) continue;
            if(contact_points_.first > centroid(2)){
                contact_points_.first = centroid(2);
                contact_points_.second.clear();
                for(int j=0;j<4;j++){
                    if(j==ignore) continue;
                    contact_points_.second.emplace_back(contact_point[j]);
                }
                normal_ = normal;
                center_pos_ = (vehicle_point[0] + vehicle_point[2]) * 0.5;
                centroid_pos_ = centroid;
            }
        }
        return !(contact_points_.first == DBL_MAX);
    }

    bool FindContactPoint3(const vector<Vector3>& rectangle_ground){
        for(int i=0;i<4;i++){
            int ignore = i;
            vector<Vector3> contact_point = rectangle_ground;
            contact_point[i] = contact_point[(i+1)%4] + contact_point[(i+3)%4] 
                               - contact_point[(i+2)%4];
            double height_differ;
            //0号点
            height_differ = (contact_point[0](2)*2.0 - contact_point[3](2)) - ground_[0](2);
            if(height_differ < -buffer_) continue;
            //1号点
            height_differ = (contact_point[1](2)*2.0 - contact_point[2](2)) - ground_[1](2);
            if(height_differ < -buffer_) continue;
            height_differ = contact_point[i](2) - rectangle_ground[i](2);
            if(height_differ < -buffer_) continue;            
            if(abs(height_differ) <= buffer_){
                ignore = -1;
            }
            vector<Vector3> vehicle_point = {contact_point[0]*2.0 - contact_point[3],
                                             contact_point[1]*2.0 - contact_point[2],
                                             contact_point[2], contact_point[3]};
            Plane contact_plane(contact_point[(i+1)%4],contact_point[(i+2)%4],contact_point[(i+3)%4]);
            Vector3 normal = contact_plane.GetNormal();
            Vector3 centroid = GetCentroid(vehicle_point, normal);
            if(!IsInTheProjection(contact_point, centroid, ignore)) continue;
            if(contact_points_.first > centroid(2)){
                contact_points_.first = centroid(2);
                contact_points_.second.clear();
                for(int j=0;j<4;j++){
                    if(j==ignore) continue;
                    contact_points_.second.emplace_back(contact_point[j]);
                }
                normal_ = normal;
                center_pos_ = (vehicle_point[0] + vehicle_point[2]) * 0.5;
                centroid_pos_ = centroid;
            }
        }
        return !(contact_points_.first == DBL_MAX);
    }

    // 计算点 P 到直线 AB 的距离
    double DistanceToLine(const Vector3& A, const Vector3& B, const Vector3& P) {
        Vector3 v(B(0) - A(0), B(1) - A(1), B(2) - A(2));
        Vector3 AP(P(0) - A(0), P(1) - A(1), P(2) - A(2));
        return AP.cross(v).norm() / v.norm();
    }

    // 计算点 P 在直线 AB 上的垂足坐标
    Vector3 ProjectionOnLine(const Vector3& A, const Vector3& B, const Vector3& P) {
        Vector3 v = {B(0) - A(0), B(1) - A(1), B(2) - A(2)};
        Vector3 AP = {P(0) - A(0), P(1) - A(1), P(2) - A(2)};
        double t = AP.dot(v) / v.dot(v);
        return Vector3(A(0) + t * v(0), A(1) + t * v(1), A(2) + t * v(2));
    }

    double CalcNESM(const Vector3& F1, const Vector3& F2){
        Vector3 Z(0,0,1.0);
        Vector3 F1F2 = F2 - F1;
        F1F2.normalize();
        Vector3 projection = ProjectionOnLine(F2, F1, centroid_pos_);
        Vector3 R = centroid_pos_ - projection;
        double R_norm = R.norm();
        R.normalize();
        Plane p(F1, F2, F1+Z);
        // 点到平面的距离
        double distToPlane = p.DistanceToPlane(centroid_pos_);
        // 法向量
        Vector3 normal = p.GetNormal();
        if(normal.dot(R) < 0.0) normal = normal*(-1);
        // R'向量
        Vector3 RR = normal.cross(F1F2).normalize();
        return R_norm * (1 - RR.dot(R)) * (RR.dot(Z));
    }

public:
    ContactPoint(double len, double wid, 
                 double ll, double bb, double hh):length_(len), width_(wid), 
                                                  l_(ll), b_(bb), h_(hh){
        ground_.reserve(6);
        contact_points_.first = DBL_MAX;
        contact_points_.second.reserve(4);
    }

    void SetGround(const vector<Vector3>& ground){
        ground_ = ground;
        contact_points_.first = DBL_MAX;
        contact_points_.second.clear();
    }

    bool Solve(){
        vector<Vector3> rectangle_ground_1 = {ground_[0], ground_[1], ground_[3], ground_[4]};
        if(FindContactPoint1(rectangle_ground_1)){
            int len = contact_points_.second.size();
            return true;
        }else{
            vector<Vector3> rectangle_ground_2 = {ground_[0], ground_[1], ground_[2], ground_[5]};
            vector<Vector3> rectangle_ground_3 = {ground_[5], ground_[2], ground_[3], ground_[4]};
            if(FindContactPoint2(rectangle_ground_2)|| FindContactPoint3(rectangle_ground_3)){
                int len = contact_points_.second.size();
                return true;
            }else{
                cout<<"solve failed!"<<endl;
                return false;
            }
        }
    }

    bool FindMinNESM(){
        NESM_ = DBL_MAX;
        int len = contact_points_.second.size();
        for(int i=0;i<len;i++){
            double cur_NESM = CalcNESM(contact_points_.second[i], contact_points_.second[(i+1)%len]);
            NESM_ = min(NESM_, cur_NESM);
        }
        return !(NESM_ == DBL_MAX);
    }


    Vector3 GetNormal(){
        return normal_;
    }

    Vector3 GetCenterPosition(){
        return center_pos_;
    }

    Vector3 GetCentroidPosition(){
        return centroid_pos_;
    }

    vector<Vector3> GetContactPoints(){
        return contact_points_.second;
    }

    double GetNESM(){
        return NESM_;
    }

};
#endif /* CONTACT_POINT_H */