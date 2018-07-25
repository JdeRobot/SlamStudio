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


int SLAMTestBed::estimatePCAorder(MatrixXd& AA, MatrixXd& BB){
	GeneratorPCA myGeneratorPCA;
	MatrixXd AApca,BBpca (AA.rows(),3);
	int contLin=AA.rows();

	MatrixXd pcaA,pcaB;
	//B = readingB.block(0,0,contLin,3);
	//BB = readingB.block(0,0,contLin,3);//to use with Scale and PCA
	int myIndex=0;
	double myR=0,myRMax=0;
	for (int i=0; i<5;i++ ){
		 myGeneratorPCA.calculatePCAbySVD(i,AA, AApca, pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
		 myGeneratorPCA.calculatePCAbySVD(i,BB, BBpca, pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB

			FindScala myFindScala;
			Registrador myRegister; //to calculate matrix Rotation and Traslation


			// PREVIOUSLY, INSIDE calculatePCAbySVD, we calculated the difference with the centroids. No need to calculate it again
			//MatrixXd AA = A.rowwise() - A.colwise().mean();
			//MatrixXd BB = B.rowwise() - B.colwise().mean();



			//myFindScala.getScalaSVD(AA,BB);

			//
			//Vector3d myScalaSVD = myFindScala.getScalaSVD(AA,BB);
			//Vector3d myScalaSVD = myFindScala.getScalaSVD(AA,BB);
			Vector3d myScalaSVD = myFindScala.getScalaSVD(AApca,BBpca);
			std::cout <<"myScalaSVD"<<myScalaSVD<<std::endl;



			//A = A.rowwise() - A.colwise().mean();
			//B = B.rowwise() - B.colwise().mean();
			//Begin scale adaptation
			//Divide dataset B by myScalaSVD
			//MatrixXd newB (contLin,3);
			//newB = BB.block(0,0,contLin,3);
			MatrixXd newBBpca (contLin,3);
			newBBpca = BBpca.block(0,0,contLin,3);
			//newBB = BB.block(0,0,contLin,3);
			//newB = BB.block(0,0,contLin,3);

			if (myScalaSVD(0) > 1 || myScalaSVD(1) > 1 || myScalaSVD(1) > 1 ){ //if myScaleSVD is not (1,1,1)


				std::cout <<"Scale is greater than 1. Divide by Scale dataset B and datasetBB"<<myScalaSVD<<std::endl;
				/*
				 for (int i= 0; i< newB.rows(); i++){

							VectorXd aRow = newB.row(i);
							newB.row(i) << aRow(0)/myScalaSVD(0),aRow(1)/myScalaSVD(1),aRow(2)/myScalaSVD(2);


						}
                */
				for (int i= 0; i< newBBpca.rows(); i++){
					VectorXd aRowBpca = newBBpca.row(i);
					newBBpca.row(i) << aRowBpca(0)/myScalaSVD(0),aRowBpca(1)/myScalaSVD(1),aRowBpca(2)/myScalaSVD(2);

				}

				//Vector3d myScalaSVD2 = myFindScala.getScalaSVD(A,newB);
				//Vector3d myScalaSVD2BB = myFindScala.getScalaSVD(AA,newBB);
				Vector3d myScalaSVD2Bpca = myFindScala.getScalaSVD(AApca,newBBpca);
				//Vector3d myScalaSVD2 = myFindScala.getScalaSVD(AA,newB);
				//std::cout <<"myScalaSVD2. After dividing by scale dataset B"<<myScalaSVD2<<std::endl;
				std::cout <<"myScalaSVD2BB. After dividing by scale dataset Bpca"<<myScalaSVD2Bpca<<std::endl;

			}
			// end scale adaptation

	        /*
			MatrixXd ret_R,ret_T,MatrixXd , ret_Toriginal , otroBB (contLin-25,3),otroAA (contLin-25,3);

			myRegister.rigid_transform_3D(A,newB,ret_R,ret_T);
			//myRegister.rigid_transform_3D(A,B,ret_R,ret_T);
			//myRegister.rigid_transform_3D_normalized(A,B,ret_R,ret_T);
			//myRegister.rigid_transform_3D_normalized(A,newB,ret_R,ret_T);
			Vector3d centroidNewB = newB.colwise().mean();
			ret_Toriginal = -ret_R * centroidA+centroidB;
			ret_T = -ret_R * centroidA+centroidNewB;
			std::cout <<"La traslacion original es:\n"<<ret_Toriginal<<std::endl;
			std::cout <<"La traslacion sin escala es:\n"<<ret_T<<std::endl;
			std::cout <<"La rotacion sin escala es:\n"<<ret_R<<std::endl;
			Vector3d newTrasla ;
			newTrasla<< ret_T(0)*myScalaSVD(0), ret_T(1)*myScalaSVD(1), ret_T(2)*myScalaSVD(2);
			std::cout <<"La traslacion sin escala, multiplicada de nuevo por la escala es:"<<newTrasla<<std::endl;

			//std::cout <<"ret_R="<<ret_R<<std::endl;
			//std::cout <<"ret_T="<<ret_T<<std::endl;
	        */
			AjusteTiempo myFindOffset;
			//double offset = 8;
			//double offset = 0;
			//int maxLine = A.cols;
			int intervalo = 100;		//micorrelador.calcularAutocorrelacion( maxLine,intervalo, offset, A,  B);
			//myFindOffset.calculateCrossCorrelation( maxLine,intervalo, offset, A,  B);//malo
			std::cout <<"Antes de calculateOffset"<<std::endl;
			//otroBB = BB.block(25,0,contLin-25,3);
			//otroAA = AA.block(25,0,contLin-25,3);
			//mostrar valores de A y B
			//for (int i= 0; i< 30; i++){
				//std::cout <<i<<" A.col "<<A.row(i)<<" B.col "<<B.row(i)<<std::endl;

				//	}

			//for (int i= 0; i< 30; i++){
				//	std::cout <<i <<" otroAA.col "<<otroAA.row(i)<<" otroBB.col "<<otroBB.row(i)<<std::endl;

			//}
			//myFindOffset.calculateOffset( A.rows(),intervalo, offset, A,  A);//ok
			//myFindOffset.calculateOffset( A.rows(),intervalo, offset, A,  B);//ok
			//myFindOffset.calculateOffset( A.rows(),intervalo, offset, A,  newB);//ok
			//myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  newBB);//ok
			//myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  BB);//ok
			//myFindOffset.calculateOffset( maxLine,intervalo, offset, AApca,  BBpca);//ok

			// set X Y or Z to a constant value on datasetB , to check that is possible to calculate offset
			//AApca.col(0)=AApca.col(0)*0;
			//BBpca.col(0)=BBpca.col(0)*0;
			//AApca.col(1)=AApca.col(1)*0;
			//BBpca.col(1)=BBpca.col(1)*0;
			//AApca.col(2)=AApca.col(2)*0;
			//BBpca.col(2)=BBpca.col(2)*0;


			//myFindOffset.calculateOffsetXYZ( maxLine,intervalo, offset, AApca,  BBpca);//ok
			int offset=23;
			myR = myFindOffset.calculateOffsetXYZ( contLin,intervalo, offset, AApca,  newBBpca);//ok
			if (myR > myRMax){
				myRMax=myR;
				myIndex=i;
			}


			//myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  BB);//ok
			//myFindOffset.calculateOffset( otroAA.rows(),intervalo, offset, otroAA,  otroAA);//ok
			//myFindOffset.calculateOffset( maxLine,intervalo, offset, otroAA,  otroAA);//no ok
			//myFindOffset.calculateOffset( maxLine,intervalo, offset, A,  newB);//ok
			//myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  BB);//ok
			//myFindOffset.calculateCrossCorrelation( maxLine,intervalo, offset, AA,  BB);//malo
			//myFindOffset.calculateCrossCorrelation( otroAA.rows(),intervalo, offset, otroAA,  otroBB);//malo

			//std::cout <<"B"<<B<<std::endl;
	}
	std::cout <<"The best R"<<myR<<std::endl;
	std::cout <<"The best order PCa"<<myIndex<<std::endl;
	return myIndex;

}


int main(){

	//int maxLine = 500;
	int maxLine = 3000;
	Transformador myTransformador;
    SLAMTestBed mySlamTestBed;
	MatrixXd readingA (maxLine,3);
	MatrixXd readingB (maxLine,3);
	std::cout <<"1="<<std::endl;
	std::cout << std::setprecision(6) << std::fixed;
	std::ifstream infileA( "/home/tfm3/workspace/SLAMTestBed/miEntradaA.txt" );
	//std::ifstream infileA( "/home/tfm3/workspace/SLAMTestBed/miEntradaA1000.txt" );
	//inputFile >> std::setprecision(6) >> std::fixed;
	//int GNoise = 0;
	int GNoise = 1;// indicates if Gaussian Noise must be generated
	int CNoise = 0;// indicates if Cosmic Noise must be generated
	Point3D miEscala;
	//miEscala.setXYZ(3.7525,3.7525,3.7525);
	miEscala.setXYZ(3,3,3);
	//miEscala.setXYZ(2,2,2);
	Point3D miTraslacion;
	miTraslacion.setXYZ(3,3,3);

	//Creating new dataset (dataset B) similar to the first one but contaminated. Write it into a file
	double offset = 25;
	//double offset = 0;

	myTransformador.createContaminatedSequence("miEntradaA.txt","miSalidaContaminada.txt",miTraslacion,miEscala,60,'X',GNoise,CNoise,offset);
	std::cout << std::setprecision(4) << std::fixed;
	//std::ifstream infileB( "/home/tfm3/workspace/SLAMTestBed/miEntradaA.txt" );

	// Reading input file B, with new dataset contaminated
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

	MatrixXd A,AA (contLin,3);
	A = readingA.block(0,0,contLin,3);
	AA = readingA.block(0,0,contLin,3);//to use with Scale and PCA

	contLin=0;

	while ( (infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
					readingB.row(contLin)<<rx,ry,rz;

					contLin ++;

	}
	infileB.close();
	MatrixXd B,BB,AApca,BBpca (contLin,3);
	MatrixXd pcaA,pcaB;
	B = readingB.block(0,0,contLin,3);
	BB = readingB.block(0,0,contLin,3);//to use with Scale and PCA


    GeneratorPCA myGeneratorPCA;
    //Point3D miEscala2;
    //miEscala2.setXYZ(1,1,1);
    //Point3D miTraslacion2;
    //miTraslacion2.setXYZ(1,1,1);
    //myTransformador.createMatRotTraslaEscala('X',0,miTraslacion2,miEscala2);
    Vector3d centroidA = A.colwise().mean();
    Vector3d centroidB = B.colwise().mean();
    //A = A.rowwise() - centroidA.transpose();
    //B = B.rowwise() - centroidB.transpose();
    int rotatePCA = mySlamTestBed.estimatePCAorder( AA, BB);
    myGeneratorPCA.calculatePCAbySVD(rotatePCA,AA, AApca, pcaA);//A is converted to PCA. Important,inside this function also is calculated A.rowwise() - A.colwise().mean(). A is converted to a newA
    myGeneratorPCA.calculatePCAbySVD(rotatePCA,BB, BBpca, pcaB);//B is converted to PCA. Important,inside this function also is calculated B.rowwise() - B.colwise().mean(). B is converted to a newB

    FindScala myFindScala;
    Registrador myRegister; //to calculate matrix Rotation and Traslation


    // PREVIOUSLY, INSIDE calculatePCAbySVD, we calculated the difference with the centroids. No need to calculate it again
    //MatrixXd AA = A.rowwise() - A.colwise().mean();
    //MatrixXd BB = B.rowwise() - B.colwise().mean();



    //myFindScala.getScalaSVD(AA,BB);

    //
    //Vector3d myScalaSVD = myFindScala.getScalaSVD(AA,BB);
    //Vector3d myScalaSVD = myFindScala.getScalaSVD(AA,BB);
    Vector3d myScalaSVD = myFindScala.getScalaSVD(AApca,BBpca);
    std::cout <<"myScalaSVD"<<myScalaSVD<<std::endl;



    //A = A.rowwise() - A.colwise().mean();
    //B = B.rowwise() - B.colwise().mean();
    //Begin scale adaptation
    //Divide dataset B by myScalaSVD
    MatrixXd newB (contLin,3);
    newB = B.block(0,0,contLin,3);
    MatrixXd newBBpca (contLin,3);
    newBBpca = BBpca.block(0,0,contLin,3);
    //newBB = BB.block(0,0,contLin,3);
    //newB = BB.block(0,0,contLin,3);

    if (myScalaSVD(0) > 1 || myScalaSVD(1) > 1 || myScalaSVD(1) > 1 ){ //if myScaleSVD is not (1,1,1)


    	std::cout <<"Scale is greater than 1. Divide by Scale dataset B and datasetBB"<<myScalaSVD<<std::endl;
		for (int i= 0; i< newB.rows(); i++){

					VectorXd aRow = newB.row(i);
					newB.row(i) << aRow(0)/myScalaSVD(0),aRow(1)/myScalaSVD(1),aRow(2)/myScalaSVD(2);


				}

		for (int i= 0; i< newBBpca.rows(); i++){
			VectorXd aRowBpca = newBBpca.row(i);
			newBBpca.row(i) << aRowBpca(0)/myScalaSVD(0),aRowBpca(1)/myScalaSVD(1),aRowBpca(2)/myScalaSVD(2);

		}

		Vector3d myScalaSVD2 = myFindScala.getScalaSVD(A,newB);
		//Vector3d myScalaSVD2BB = myFindScala.getScalaSVD(AA,newBB);
		Vector3d myScalaSVD2Bpca = myFindScala.getScalaSVD(AApca,newBBpca);
		//Vector3d myScalaSVD2 = myFindScala.getScalaSVD(AA,newB);
		std::cout <<"myScalaSVD2. After dividing by scale dataset B"<<myScalaSVD2<<std::endl;
		std::cout <<"myScalaSVD2BB. After dividing by scale dataset Bpca"<<myScalaSVD2Bpca<<std::endl;

    }
    // end scale adaptation


    MatrixXd ret_R,ret_T,MatrixXd , ret_Toriginal , otroBB (contLin-25,3),otroAA (contLin-25,3);

    myRegister.rigid_transform_3D(A,newB,ret_R,ret_T);
    //myRegister.rigid_transform_3D(A,B,ret_R,ret_T);
    //myRegister.rigid_transform_3D_normalized(A,B,ret_R,ret_T);
    //myRegister.rigid_transform_3D_normalized(A,newB,ret_R,ret_T);
    Vector3d centroidNewB = newB.colwise().mean();
    ret_Toriginal = -ret_R * centroidA+centroidB;
    ret_T = -ret_R * centroidA+centroidNewB;
    std::cout <<"La traslacion original es:\n"<<ret_Toriginal<<std::endl;
    std::cout <<"La traslacion sin escala es:\n"<<ret_T<<std::endl;
    std::cout <<"La rotacion sin escala es:\n"<<ret_R<<std::endl;
    Vector3d newTrasla ;
    newTrasla<< ret_T(0)*myScalaSVD(0), ret_T(1)*myScalaSVD(1), ret_T(2)*myScalaSVD(2);
    std::cout <<"La traslacion sin escala, multiplicada de nuevo por la escala es:"<<newTrasla<<std::endl;

	//std::cout <<"ret_R="<<ret_R<<std::endl;
    //std::cout <<"ret_T="<<ret_T<<std::endl;

    AjusteTiempo myFindOffset;
    //double offset = 8;
    //double offset = 0;
	//int maxLine = A.cols;
	int intervalo = 100;		//micorrelador.calcularAutocorrelacion( maxLine,intervalo, offset, A,  B);
	//myFindOffset.calculateCrossCorrelation( maxLine,intervalo, offset, A,  B);//malo
	std::cout <<"Antes de calculateOffset"<<std::endl;
	otroBB = BB.block(25,0,contLin-25,3);
    otroAA = AA.block(25,0,contLin-25,3);
    //mostrar valores de A y B
	for (int i= 0; i< 30; i++){
		std::cout <<i<<" A.col "<<A.row(i)<<" B.col "<<B.row(i)<<std::endl;

			}

	for (int i= 0; i< 30; i++){
			std::cout <<i <<" otroAA.col "<<otroAA.row(i)<<" otroBB.col "<<otroBB.row(i)<<std::endl;

	}
	myFindOffset.calculateOffset( A.rows(),intervalo, offset, A,  A);//ok
	myFindOffset.calculateOffset( A.rows(),intervalo, offset, A,  B);//ok
	myFindOffset.calculateOffset( A.rows(),intervalo, offset, A,  newB);//ok
	//myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  newBB);//ok
	myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  BB);//ok
	myFindOffset.calculateOffset( maxLine,intervalo, offset, AApca,  BBpca);//ok

	// set X Y or Z to a constant value on datasetB , to check that is possible to calculate offset
	//AApca.col(0)=AApca.col(0)*0;
	//BBpca.col(0)=BBpca.col(0)*0;
	//AApca.col(1)=AApca.col(1)*0;
	//BBpca.col(1)=BBpca.col(1)*0;
	//AApca.col(2)=AApca.col(2)*0;
	//BBpca.col(2)=BBpca.col(2)*0;


	myFindOffset.calculateOffsetXYZ( maxLine,intervalo, offset, AApca,  BBpca);//ok
	myFindOffset.calculateOffsetXYZ( maxLine,intervalo, offset, AApca,  newBBpca);//ok


	//myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  BB);//ok
	//myFindOffset.calculateOffset( otroAA.rows(),intervalo, offset, otroAA,  otroAA);//ok
	//myFindOffset.calculateOffset( maxLine,intervalo, offset, otroAA,  otroAA);//no ok
	//myFindOffset.calculateOffset( maxLine,intervalo, offset, A,  newB);//ok
	//myFindOffset.calculateOffset( maxLine,intervalo, offset, AA,  BB);//ok
	//myFindOffset.calculateCrossCorrelation( maxLine,intervalo, offset, AA,  BB);//malo
	//myFindOffset.calculateCrossCorrelation( otroAA.rows(),intervalo, offset, otroAA,  otroBB);//malo

	//std::cout <<"B"<<B<<std::endl;











}
