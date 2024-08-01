#ifndef CONVERSION_H_
#define CONVERSION_H_

#include <Eigen/Geometry>

namespace sodf {

double toRadians(double degrees);
double toDegrees(double radians);

/*
yaw:
    A yaw is a counterclockwise rotation of alpha about the  z-axis. The rotation matrix is given by

    R_z

    |cos(alpha) -sin(alpha) 0|
    |sin(apha)   cos(alpha) 0|
    |    0            0     1|

pitch:
    R_y
    A pitch is a counterclockwise rotation of  beta about the  y-axis. The rotation matrix is given by

    |cos(beta)  0   sin(beta)|
    |0          1       0    |
    |-sin(beta) 0   cos(beta)|

roll:
    A roll is a counterclockwise rotation of  gamma about the  x-axis. The rotation matrix is given by
    R_x
    |1          0           0|
    |0 cos(gamma) -sin(gamma)|
    |0 sin(gamma)  cos(gamma)|



    It is important to note that   R_z R_y R_x performs the roll first, then the pitch, and finally the yaw
    Roration matrix: R_z*R_y*R_x
*/
// https://ros-developer.com/2017/11/18/roll-pitch-yaw-using-eigen-kdl-frame/
Eigen::Isometry3d RPY(double roll, double pitch, double yaw);

}  // namespace sodf

#endif  // CONVERSION_H_
