#ifndef GENERATORPCA_H
#define GENERATORPCA_H
#include <iostream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <math.h>
#include <cstdlib>
#include "Eigen/Dense"
#include "Eigen/SVD"
#include <limits>
#include <stdbool.h>
using namespace Eigen;
//using namespace std;

class GeneratorPCA {
private:
	MatrixXd m;
    MatrixXd pcaA;
    MatrixXd pcaB;
public:
   GeneratorPCA ();
   //void genera2Series(int maxLine, double proporcionFrecuencia,double offset, MatrixXd& A, MatrixXd& B);
   //void calcularAutocorrelacion(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   //void calcularAutocorrelacion2(int maxLine,int intervalo, double offset, MatrixXd&A, MatrixXd&B);
   //void calcularAutocorrelacion3(char coordinate, int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2);
   void calculatePCA(int maxLine, MatrixXd& A1);
   void calculatePCAbySVD(int maxLine,MatrixXd& A1);
   void calculatePCAbySVD(int rotation,MatrixXd& A, MatrixXd& A2, MatrixXd& PCA);
   void rotatePCA (int cont,MatrixXd& PCA);
   MatrixXd getPcaA();
   MatrixXd getPcaB();
   void setPcaA(MatrixXd aMatrix);
   void setPcaB(MatrixXd aMatrix);
};
#endif
