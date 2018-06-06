#ifndef FINDSCALA_H
#define FINDSCALA_H
#include <iostream>
#include "Eigen/Dense"
using namespace Eigen;
//using namespace std;

class FindScala {
private:
	MatrixXd m;

public:
   FindScala () ;
   void demoPresentaMatriz(); // Constructor with default arguments
   void rigid_transform_3D (MatrixXd A, MatrixXd B, MatrixXd& R, MatrixXd& t);
   void getScalaRansac(MatrixXd AA, MatrixXd BB, int ContLin);
   Vector3d getScalaEigenValues(MatrixXd AA, MatrixXd BB);
   Vector3d getScalaSVD(MatrixXd AA, MatrixXd BB);

};
#endif
