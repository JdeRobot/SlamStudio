#ifndef REGISTRADORHORN_H
#define REGISTRADORHORN_H
#include <iostream>
#include "Eigen/Dense"
using namespace Eigen;
//using namespace std;

class RegistradorHorn {
private:
	MatrixXd m;


public:
   RegistradorHorn () ;
   void demoPresentaMatriz(); // Constructor with default arguments
   void rigid_transform_3D (MatrixXd A, MatrixXd B, MatrixXd& R, MatrixXd& t);
   void align(MatrixXd model, MatrixXd data, MatrixXd& rot, MatrixXd& trans);

};
#endif
