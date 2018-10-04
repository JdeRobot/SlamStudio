#ifndef AJUSTETIEMPO_H
#define AJUSTETIEMPO_H
#include <iostream>
#include "Eigen/Dense"
using namespace Eigen;
//using namespace std;

class AjusteTiempo {
private:
	MatrixXd m;

public:
   AjusteTiempo () ;
   void demoPresentaMatriz(); // Constructor with default arguments
   void rigid_transform_3D (MatrixXd A, MatrixXd B, MatrixXd& R, MatrixXd& t);
   void getScalaRansac(MatrixXd AA, MatrixXd BB, int ContLin);
   void genera2Series(int maxLine, double proporcionFrecuencia,double offset, MatrixXd& A, MatrixXd& B);
   void calcularAutocorrelacion(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   void calcularAutocorrelacion2(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   void calculateCrossCorrelation(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   void calcularAutocorrelacion3(char coordinate, int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2);
   void calculateOffset(int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2);
   double calcularAutocorrelacion4(int maxLine, int intervalo,double offset, MatrixXd& A, MatrixXd& B);
   //double calculateOffsetXYZ(int maxLine, int interval, double offset, MatrixXd& A1,MatrixXd& B2);
   double calculateOffsetXYZ(int maxLine, int interval, double offset, double& offsetEstimated, MatrixXd A1,MatrixXd B2);
   Vector3d getScalaEigenValues(MatrixXd AA, MatrixXd BB);
   Vector3d getScalaSVD(MatrixXd AA, MatrixXd BB);

};
#endif
