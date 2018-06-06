#include <iostream>
#include <ctime>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include "Eigen/Dense"
using namespace Eigen;
#include <limits>
#include <stdbool.h>
//#include "../RegistradorHorn/RegistradorHorn.h"
#include "RegistradorHorn.h"

//#include "Eigen/Dense"

int main( int argc, char** argv )
{
	//std::cout << std::setprecision(6) << std::fixed;

    std::ofstream out( "/home/tfm3/workspace/RegistradorRansac/miSalidaRegistro.txt" );
    out << std::setprecision(6) << std::fixed;
	std::ifstream infile("/home/tfm3/workspace/RegistradorRansac/miEntrada.txt");
	//std::string line;
	//infile >> std::setprecision(6) >> std::fixed;
	//MatrixXd A (3000,3);
	std::cout <<"antes de declarar la variable"<<std::endl;
	MatrixXd readingA (6000,3);//size is big enough to store an unknown number of data rows from file
	//MatrixXd readingA;
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	std::cout <<"antes de leer el archivo"<<std::endl;
	while ( infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4 ){
	//while ( infile >> rx >> ry >> rz  ){
			//std::cout << "You said.... contLin"<< contLin << timestamp << " " << rx << " " << ry << " " << rz << " " << q1 << " " << q2 << " " << q3 << " " << q4 << "\n";
			//VectorXd myVector (rx,ry,rz);
			//readingA.row(contLin)= myVector;
			readingA.row(contLin)<< rx,ry,rz;
			contLin ++;

	}
	infile.close();
	std::cout <<"contLin="<<contLin<<std::endl;
	MatrixXd A (contLin,3);
	//A = readingA.block<contLin,3>(0,0);
	A = readingA.block(0,0,contLin,3);


	std::cout << ">>>>>>>CIERRO INFILE\n"<<std::endl;

	std::ifstream infile2("/home/tfm3/workspace/RegistradorRansac/miEntradaModificada.txt");
	//std::string line;
    //infile >> std::setprecision(6) >> std::fixed;
	//MatrixXd A (3000,3);
	std::cout <<"antes de declarar la variable"<<std::endl;
	MatrixXd readingB (6000,3);//size is big enough to store an unknown number of data rows from file
	//MatrixXd readingA;
	contLin=0;
	timestamp=0,rx=0,ry=0,rz=0,q1,q2,q3,q4=0;
	std::cout <<"antes de leer el archivo"<<std::endl;
	while ( infile2 >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4 ){
	        readingB.row(contLin)<< rx,ry,rz;
	        contLin ++;

	}
	infile2.close();
	std::cout <<"contLin="<<contLin<<std::endl;
	MatrixXd B (contLin,3);
	B = readingB.block(0,0,contLin,3);


	std::cout << ">>>>>>>CIERRO INFILE\n"<<std::endl;

int iterations = 0;
int maxIterations = 3;
MatrixXd FinalRotation;
Vector3d FinalTraslation;
double bestError = 99999;
unsigned t0, t1;
t0 = clock();
while (iterations < maxIterations){ // Comenzamos el bucle RANSANC
    iterations ++;
	//std::cout << "B "<<B<<std::endl;
	int nvalores = 20;
	// Select 30 points inliers,
	MatrixXd mInliers (nvalores,3);
	MatrixXd mData (nvalores,3);
	int i=0;
	bool visitados[contLin] ={ false};
    int aInlierIdx =0;
	while (i < nvalores) { // Seleccionamos aleatoriamente los primeros supuestos inliers

		//get random value
		//srand(time(0));
		//srand(aInlierIdx);
		//aInlierIdx = rand() % contLin;
		aInlierIdx=1+(int) (3000.0*rand()/(RAND_MAX+1.0));
		//if (visitados[aInlierIdx]==false) {
			visitados[aInlierIdx]=true;
			std::cout<<i << " aInlierIdx "<< aInlierIdx	<<std::endl;
			mInliers.row(i)= B.row(aInlierIdx);
			mData.row(i)= A.row(aInlierIdx);
			i++;
		//}
	}


	//std::cout << "mInliers "<<mInliers<<std::endl;

	RegistradorHorn miRegistradorHorn;
	MatrixXd ret_R,ret_t,second_xyz_aligned;
	//miRegistradorHorn.align(B.transpose(),A.transpose(),ret_R,ret_t);
	miRegistradorHorn.align(mInliers.transpose(),mData.transpose(),ret_R,ret_t); // Estimamos matriz rotacion traslacion

	second_xyz_aligned = ret_R * mInliers.transpose();

	MatrixXd mTransMatrix (second_xyz_aligned.rows(),second_xyz_aligned.cols()),  model_aligned2;
	for (int j=0; j< second_xyz_aligned.cols(); j++ ) {

		mTransMatrix.col(j) << ret_t(0), ret_t(1), ret_t(2);

	}
	//std::cout << "mTransMatrix \n"<< mTransMatrix <<std:: endl;
	model_aligned2=second_xyz_aligned+mTransMatrix;


	//std::cout << "mData \n"<< mData <<std:: endl;
	//std::cout << "model_aligned2 \n"<< model_aligned2 <<std:: endl;
	//MatrixXd alignment_error = model_aligned - data
	MatrixXd mErr = model_aligned2.transpose() - mData;
	int numCols = model_aligned2.cols();

	//for (int w=0;w<numCols; w++){
	//	out<<w <<" "<< model_aligned2(0,w) <<" "<< model_aligned2(1,w) <<" "<< model_aligned2(2,w) <<" "<< w <<" "<< w <<" "<< w <<" "<< w<<std:: endl;
	//}
	//out.close();
	// calculo la media del error de la diferencia entre los inliers y los datos originales
	//std::cout << "mErr  \n"<< mErr <<std:: endl;
	// calculo el valor absoluto
	//std::cout << "mErr abs \n"<< mErr.cwiseAbs() <<std:: endl;
	// calculo la media por columnas

	Vector3d meanInliers = (mErr.cwiseAbs()).colwise().mean();
	//std::cout << "meanInliers \n"<< meanInliers <<std:: endl;

	//Una vez estimado el modelo con los primeros inliers (matriz Rotacion y traslacion) intentaré encontrar cuales del resto de puntos escajarian en el modelo

    for (int k = 0; k<contLin; k ++ ){

    	if (!visitados[k]) {

    		Vector3d newInlier=ret_R*B.row(k).transpose();//aplicamos al punto B la rotación estimada
    		newInlier = newInlier + ret_t; //sumamos la traslacion estimada
    		//std::cout << "newInlier "<< newInlier(0)<<" "<< newInlier(1)<<" "<<newInlier(2)<<std::endl;
    		Vector3d vErrorInlier = (A.row(k).transpose() - newInlier).cwiseAbs();
    		//std::cout << "vErrorInlier "<< vErrorInlier(0)<<" "<< vErrorInlier(1)<<" "<<vErrorInlier(2)<<std::endl;
    		if (vErrorInlier(0)<0.0025 && vErrorInlier(1)<0.0025 && vErrorInlier(2)<0.0025) {
    			//std::cout << "vErrorInlier "<< vErrorInlier(0)<<" "<< vErrorInlier(1)<<" "<<vErrorInlier(2)<<std::endl;
				//std::cout << "newInlier "<< newInlier <<std::endl;
				visitados[k]=true;

    		}


    	}
    }
    int contTotalInliers =0;
    for (int c=0;c< contLin;c++){
    	if (visitados[c]) contTotalInliers ++;
    }
    MatrixXd mFinalInliers (contTotalInliers,3);
    MatrixXd mFinalData (contTotalInliers,3);
    int idxFinalInlier=0;
    for (int g = 0; g < contLin; g++){

    	if (visitados[g]){
    		mFinalInliers.row(idxFinalInlier)=B.row(g);
    		mFinalData.row(idxFinalInlier)=A.row(g);
    		idxFinalInlier++;
    	}
    }
    std::cout << "Num final Inliers "<< idxFinalInlier <<std::endl;

    miRegistradorHorn.align(mFinalInliers.transpose(),mFinalData.transpose(),ret_R,ret_t);


    second_xyz_aligned = ret_R * mFinalInliers.transpose();

    MatrixXd mTransMatrixFinal (second_xyz_aligned.rows(),second_xyz_aligned.cols()), modelAligned_final, BestModelAligned;
    for (int h=0; h< second_xyz_aligned.cols(); h++ ) {

    	mTransMatrixFinal.col(h) << ret_t(0), ret_t(1), ret_t(2);

    }

    //std::cout << "mTransMatrix \n"<< mTransMatrix <<std:: endl;
    modelAligned_final=second_xyz_aligned+mTransMatrixFinal;


    //std::cout << "mData \n"<< mFinalData <<std:: endl;

    //std::cout << "model_final \n"<< modelAligned_final <<std:: endl;

    	//MatrixXd alignment_error = model_aligned - data
    mErr = modelAligned_final.transpose() - mFinalData;


    MatrixXd mErr2 = mErr.array().pow(2);
    double errorNumber = sqrt(mErr2.sum()/mErr.rows());

    //std::cout << "err2 "<< err2 <<std::endl;
    std::cout << "RMSE= "<< errorNumber <<std::endl;

    if (bestError > errorNumber){
    	FinalRotation=ret_R;
    	FinalTraslation=ret_t;
    	bestError = errorNumber;
    }


}
t1 = clock();
std::cout << "BestError= "<< bestError <<std::endl;
std::cout << "FinalRotation= "<< FinalRotation <<std::endl;
std::cout << "FinalTraslation= "<< FinalTraslation <<std::endl;


// Una vez tenemos el mejor modelo, aplicamos a los puntos la mejor estimacion de la matriz de rotacion y traslacion a los
//  y guardamos en un fichero la transformación resultante de dichos puntos
MatrixXd Final_xyz_aligned;
Final_xyz_aligned = FinalRotation * B.transpose() ;
MatrixXd mFinalTransMatrix (Final_xyz_aligned.rows(),Final_xyz_aligned.cols()),  Final_model_aligned2;
        for (int j=0; j< Final_xyz_aligned.cols(); j++ ) {

        	mFinalTransMatrix.col(j) << FinalTraslation(0), FinalTraslation(1), FinalTraslation(2);

        }
        std::cout << "mFinalTransMatrix \n"<< mFinalTransMatrix <<std:: endl;
        Final_model_aligned2=Final_xyz_aligned+mFinalTransMatrix;

        //std::cout << "Final_model_aligned2 \n"<< Final_model_aligned2 <<std:: endl;
        std::cout << "Final_model_aligned2.rows() \n"<< Final_model_aligned2.rows() <<std:: endl;
        std::cout << "Final_model_aligned2.cols() \n"<< Final_model_aligned2.cols() <<std:: endl;
        //MatrixXd alignment_error = model_aligned - data

    int numCols = Final_model_aligned2.cols();

    for (int w=0;w<numCols; w++){
    	out<<w <<" "<< Final_model_aligned2(0,w) <<" "<< Final_model_aligned2(1,w) <<" "<< Final_model_aligned2(2,w) <<" "<< w <<" "<< w <<" "<< w <<" "<< w<<std:: endl;
    }
    out.close();

    double time = (double(t1-t0)/CLOCKS_PER_SEC);
    std::cout << "Execution Time: " << time << std::endl;

}
