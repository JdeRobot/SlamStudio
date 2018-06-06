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
using namespace Eigen;
#include <limits>
#include <stdbool.h>
#include "SLAMTestBed.h"
#include "GeneratorPCA.h"  //To calculate PCA
//#include "../AjusteTiempo/AjusteTiempo.h"
#include "FindScala.h"    // to Find Scale
#include "Transformador.h"// to contaminate groundTruth
#include "Registrador.h"//to calculate Rotation Traslation Matrix
#include "AjusteTiempo.h"// to calculate Offset

SLAMTestBed::SLAMTestBed(){
	std::cout<< "constructor por defecto Interpolator" <<std::endl;
}



int main(){

	int maxLine = 2500;
	Transformador myTransformador;

	MatrixXd readingA (maxLine,3);
	MatrixXd readingB (maxLine,3);
	std::cout <<"1="<<std::endl;
	std::cout << std::setprecision(4) << std::fixed;
	std::ifstream infileA( "/home/tfm3/workspace/SLAMTestBed/miEntradaA.txt" );
	//inputFile >> std::setprecision(6) >> std::fixed;
	int GNoise = 0;
	Point3D miEscala;
	miEscala.setXYZ(1,1,1);
	Point3D miTraslacion;
	miTraslacion.setXYZ(1,1,1);
	myTransformador.createContaminatedSequence("miEntradaA.txt","miSalidaContaminada.txt",miTraslacion,miEscala,0,'X',GNoise);
	std::cout << std::setprecision(4) << std::fixed;
	//std::ifstream infileB( "/home/tfm3/workspace/SLAMTestBed/miEntradaA.txt" );
	std::ifstream infileB( "miSalidaContaminada.txt" );



	//inputFile >> std::setprecision(6) >> std::fixed;
	std::cout <<"2="<<std::endl;
	std::cout << std::setprecision(4) << std::fixed;
	std::ofstream outA( "/home/tfm3/workspace/SLAMTestBed/miSalidaA.txt" );
	outA << std::setprecision(4) << std::fixed;
	std::cout << std::setprecision(4) << std::fixed;

	std::ofstream outB( "/home/tfm3/workspace/SLAMTestBed/miSalidaB.txt" );
	outB << std::setprecision(4) << std::fixed;

	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	int contLin=0;
	std::cout <<"4="<<std::endl;
	while ( (infileA >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
				//std::cout <<"contLin="<<contLin<<std::endl;
				readingA.row(contLin)<< rx,ry,rz;
				//std::cout <<"contLin="<<contLin<<std::endl;
				contLin ++;


	}
	infileA.close();
	MatrixXd A (contLin,3);
	A = readingA.block(0,0,contLin,3);

	contLin=0;

	while ( (infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
					readingB.row(contLin)<<rx,ry,rz;

					contLin ++;

	}
	infileB.close();
	MatrixXd B (contLin,3);
	B = readingB.block(0,0,contLin,3);


    //GeneratorPCA myGeneratorPCA;
    //Point3D miEscala2;
    //miEscala2.setXYZ(1,1,1);
    //Point3D miTraslacion2;
    //miTraslacion2.setXYZ(1,1,1);
    //myTransformador.createMatRotTraslaEscala('X',0,miTraslacion2,miEscala2);

    //myGeneratorPCA.calculatePCAbySVD(A);
    //myGeneratorPCA.calculatePCAbySVD(B);

    //FindScala myFindScala;
    Registrador myRegister; //to calculate matrix Rotation and Traslation

    //MatrixXd AA = A.rowwise() - A.colwise().mean();
    //MatrixXd BB = B.rowwise() - B.colwise().mean();

    //myFindScala.getScalaSVD(AA,BB);


    MatrixXd ret_R,ret_T;

    //myRegister.rigid_transform_3D(A,B,ret_R,ret_T);
	//std::cout <<"ret_R="<<ret_R<<std::endl;
    //std::cout <<"ret_T="<<ret_T<<std::endl;

    AjusteTiempo myFindOffset;
    double offset = 40;
	//int maxLine = A.cols;
	int intervalo = 100;		//micorrelador.calcularAutocorrelacion( maxLine,intervalo, offset, A,  B);
	//myFindOffset.calculateCrossCorrelation( maxLine,intervalo, offset, A,  B);
	std::cout <<"Antes de calculateOffset"<<std::endl;
	myFindOffset.calculateOffset( maxLine,intervalo, offset, A,  B);











}
