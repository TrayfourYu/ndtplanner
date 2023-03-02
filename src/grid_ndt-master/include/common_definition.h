//
// Created by yangt on 2019/12/2.
//

#ifndef TERRAIN_UTILIS_COMMON_DEFINITION_HPP
#define TERRAIN_UTILIS_COMMON_DEFINITION_HPP

#include<Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef double Real;

template<int Dim>
using Mat=Eigen::Matrix<Real, Dim, Dim>;

template<int Dim>
using Vector=Eigen::Matrix<Real, Dim, 1>;

typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> VectorX;
typedef Vector<3> Point3;
typedef Vector<2> Point2;
typedef Eigen::Quaternion<Real> Quater;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> IdMatrixX;

struct Pose3d
{
    double x, y, z, qx=0, qy=0, qz=0, qw=1;
    double yaw_ = 0.0;
    Pose3d()=default;
    Pose3d(double xx, double yy, double zz, 
           double qxx=0, double qyy=0, double qzz=0, double qww=1):x(xx), y(yy), z(zz),
                                                                   qx(qxx), qy(qyy), qz(qzz), qw(qww){
        this->getYaw();
    };
    Pose3d msgToPose(const geometry_msgs::Pose &msg) {
        return Pose3d(msg.position.x, msg.position.y, msg.position.z,
                      msg.orientation.x, msg.orientation.y,msg.orientation.z,msg.orientation.w);
    }
    void getYaw(){
        tf::Quaternion rot(this->qx,this->qy,this->qz,this->qw);
        tf::Matrix3x3 m(rot);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        this->yaw_ = yaw;
    }
    double EuclidDis2D(const Pose3d &other) const {
        return sqrt(pow(this->x - other.x, 2)
                  + pow(this->y - other.y, 2));
    }
};

struct State {
  Pose3d pose;
  Point3 n;
  Point3 alpha;
  Point3 beta;
  double k;
  double s;
  Vector<3> euler_angle;
};

#endif //TERRAIN_UTILIS_COMMON_DEFINITION_HPP
