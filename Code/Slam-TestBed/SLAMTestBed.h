#ifndef SLAMTESTBED_H
#define SLAMTESTBED_H
#include <iostream>
#include "Eigen/Dense"
//#include "../AjusteTiempo/AjusteTiempo.h"
using namespace Eigen;
//using namespace std;

class SLAMTestBed {
private:
	MatrixXd m;

public:
   SLAMTestBed ();
   //void genera2Series(int maxLine, double proporcionFrecuencia,double offset, MatrixXd& A, MatrixXd& B);
   //void calcularAutocorrelacion(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   //void calcularAutocorrelacion2(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   //void calcularAutocorrelacion3(char coordinate, int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2);
   void readMatrixFromFile(int maxLine,char* fileName, MatrixXd& A);
   void writeMatrixFromFile(int maxLine,char* fileName, MatrixXd& A);




};
#endif
