
#ifndef TRANSFORMADOR_H
#define TRANSFORMADOR_H
#include "../Point3D.h"
#include "../Eigen/Geometry" //for quaternion
#include "../Eigen/Dense"
#include "../Eigen/SVD"
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <limits>

#include "../Interpolator/Interpolator.h"

using namespace Eigen;

class Transformador {
private:
	int contCosmicNoise=0;
	double initTime = 0.0; // I will store the initial value for the timestamp
	double frequency=0.0;// It will store the value for the frequency. Example 0.3  then timestamp will be 1.3 , 1.6 , 1.9
	double offset = 0.0;// It will store the value of the offset. Example initTime + offset

	double timestamp,rx,ry,rz,q1,q2,q3,q4;
	//Point3D myPoint3D (3.0,3.0,3.0);
	Point3D myPoint3D;
	double matRot [4] [4] = {
	    {0.0}
	};
    //MatrixXd matRotTraslaEigen;
	double matRotTrasla [4][4] = {
		{1.0, 0.0 , 0.0, 0.0},
		{0.0, 1.0 , 0.0, 0.0},
		{0.0, 0.0 , 1.0, 0.0},
		{0.0, 0.0 , 0.0, 1.0}

	};


    Matrix3d matRot_toQuaternion;//On this matrix we will store Rotation , that will be converted to a quaternion

	double traslacion [4] [4] = {

	    {1.0, 0.0 , 0.0, 0.0},
	    {0.0, 1.0 , 0.0, 0.0},
	    {0.0, 0.0 , 1.0, 0.0},
	    {0.0, 0.0 , 0.0, 1.0}

	};

	double identidad [4] [4] = {

	    {1.0, 0.0 , 0.0, 0.0},
	    {0.0, 1.0 , 0.0, 0.0},
	    {0.0, 0.0 , 1.0, 0.0},
	    {0.0, 0.0 , 0.0, 1.0}

	};

	double punto [4] [1] = {
	    {10.0},
	    {10.0},
	    {10.0},
	    {1.0}
	};
	//float punto [4] [1] = {0.0};
	double newPunto [4]  [1] = { {0.0},
		    {0.0},
		    {0.0},
		    {0.0} };


public:
	void createMatRotTrasla ( char eje , float angulo, Point3D trasla) ;
    void createMatRot (float radX,float radY, float radZ);
	Point3D multiplicaMatrizPunto (double matriz [4][4], double unPunto[4][1], int colsMatriz, int rowsMatriz, int colsUnPunto , int rowsUnPunto);
	Point3D multiplicaMatrizPunto (double unPunto[4][1], int colsUnPunto , int rowsUnPunto);
	void multiplicaMatrizPorMatriz (double matrizA [4][4], double matrizB[4][4], int colsMatrizA, int rowsMatrizA, int colsMatrizB , int rowsMatrizB);
	void setPoint3D (Point3D aPoint3D);
	void setTraslacion (Point3D aPoint3D);
	void displayMatriz (double matriz [4][4], int matriz_rows, int matriz_cols);
	void displayMatrizRotTrasla ();
	double generateGaussianNoise(double mu, double sigma,int CNoise);
    void createMatRotTraslaEscala ( float rotaX, float rotaY, float rotaZ, Point3D aPoint3D,Point3D aScala );
	void setFrequency(double myFrequency);
    double getFrequency();
    double getOffset();
    void setOffset(double myOffset);
    void setInitTime(double initialValue);
    double getInitTime();
    //void createContaminatedSequence(char* inputFileName,char* outputFileName,int GNoise);
    void createContaminatedSequence(char* inputFileName,char* outputFileName,Point3D traslacion,Point3D escala,double radX,double radY,double radZ, int GNoise,int CNoise,double offset,int freqType,double frequency, double gnoise_value);
    Matrix3d getMatRot_toQuaternion();
    void setMatRot_toQuaternion(Matrix3d aMatrix);


};
#endif
