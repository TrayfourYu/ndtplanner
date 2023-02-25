//
// Created by yangt on 2019/12/2.
//

#ifndef TERRAIN_UTILIS_COMMON_DEFINITION_HPP
#define TERRAIN_UTILIS_COMMON_DEFINITION_HPP

#include<Eigen/Dense>

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
    double x, y, z;
    Pose3d()=default;
    Pose3d(double xx, double yy, double zz):x(xx), y(yy), z(zz){};
    Pose3d msgToPose(const geometry_msgs::Pose &msg) {
        return Pose3d(msg.position.x, msg.position.y, msg.position.z);
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
