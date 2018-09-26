#ifndef REGISTRADOR_H
#define REGISTRADOR_H
#include <iostream>
#include "Eigen/Dense"
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

};
#endif
