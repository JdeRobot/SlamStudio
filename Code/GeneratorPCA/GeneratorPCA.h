#ifndef GENERATORPCA_H
#define GENERATORPCA_H
#include <iostream>
#include "Eigen/Dense"
using namespace Eigen;
//using namespace std;

class GeneratorPCA {
private:
	MatrixXd m;

public:
   GeneratorPCA ();
   //void genera2Series(int maxLine, double proporcionFrecuencia,double offset, MatrixXd& A, MatrixXd& B);
   //void calcularAutocorrelacion(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   //void calcularAutocorrelacion2(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   //void calcularAutocorrelacion3(char coordinate, int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2);
   void calculatePCA(int maxLine, MatrixXd& A1);
   void calculatePCAbySVD(int maxLine,MatrixXd& A1);
   void calculatePCAbySVD(MatrixXd& A1);

};
#endif
