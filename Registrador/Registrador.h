#ifndef REGISTRADOR_H
#define REGISTRADOR_H
#include <iostream>
#include "Eigen/Dense"
#include "Eigen/Geometry"
using namespace Eigen;
//using namespace std;

class Registrador {
private:
	MatrixXd m;

public:
   Registrador () ;
   void demoPresentaMatriz(); // Constructor with default arguments
   void rigid_transform_3D (MatrixXd A, MatrixXd B, MatrixXd& R, MatrixXd& t);
   void rigid_transform_3D_normalized (MatrixXd A, MatrixXd B, MatrixXd& R, MatrixXd& t);

   // this method apply both rotation and traslation estimated matrix over a dataset
   void applyTransformationsOverData(MatrixXd dataInitial,MatrixXd& dataEstimated, MatrixXd rotationEstimated, MatrixXd traslationEstimated);
   void applyTransformationsOverQuaternion(MatrixXd dataInitialQuaternion,MatrixXd& dataEstimated, MatrixXd rotationEstimated);
   Matrix3d matRot_toQuaternion; //important rotMatrix should be 3x3 to convert to a quaternion
   Matrix3d getMatRot_toQuaternion();
};
#endif
