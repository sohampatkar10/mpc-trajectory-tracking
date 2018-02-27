#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

class FlatSpaceConversions {
public:
  FlatSpaceConversions(){};

  tf::Transform flat2tf(vector<double>& flatStates) {
    Eigen::Vector3d y2(flatStates[6], flatStates[7], flatStates[8]+9.81);

    std::cout << "y2 : " << y2 << endl;

    double T = y2.norm();

    Eigen::Vector3d Rz = y2/T;
    Eigen::Vector3d yawComp(cos(flatStates[12]), sin(flatStates[12]), 0.0);
    Eigen::Vector3d Ry = Rz.cross(yawComp)/(Rz.cross(yawComp).norm());
    Eigen::Vector3d Rx = Ry.cross(Rz);

    Eigen::Matrix3d R;
    R.block<3,1>(0,0) = Rx; R.block<3,1>(0,1) = Ry; R.block<3,1>(0,2) = Rz;


    tf::Matrix3x3 tfR; tf::matrixEigenToTF(R, tfR);
    return(tf::Transform(tfR, tf::Vector3(flatStates[0], flatStates[1], flatStates[2])));
  }
};