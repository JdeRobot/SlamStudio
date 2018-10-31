#include "Registrador.h"
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <cstdlib>

#include <limits>


//#include "Eigen/Dense"

int main( int argc, char** argv )
{
	//std::cout << std::setprecision(6) << std::fixed;
    std::ofstream out( "miSalidaRegistro.txt" );
    //out << std::setprecision(6) << std::fixed;


	std::ifstream infile("/home/tfm3/workspace/Registrador/miEntrada.txt");
	//std::string line;
    //infile >> std::setprecision(6) >> std::fixed;
	//MatrixXd A (3000,3);

	MatrixXd readingA (6000,3);//size is big enough to store an unknown number of data rows from file
	//MatrixXd readingA;
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	std::cout <<"antes de leer el archivo"<<std::endl;
	while ( infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4 ){
	        //std::cout << "You said.... contLin"<< contLin << timestamp << " " << rx << " " << ry << " " << rz << " " << q1 << " " << q2 << " " << q3 << " " << q4 << "\n";
	        //VectorXd myVector (rx,ry,rz);
	        //A.row(contLin)= myVector;
	        readingA.row(contLin)<< rx,ry,rz;
	        contLin ++;

	}
	infile.close();
	std::cout <<"contLin="<<contLin<<std::endl;
	MatrixXd A (contLin,3);
	//A = readingA.block<contLin,3>(0,0);
	A = readingA.block(0,0,contLin,3);


	std::cout << ">>>>>>>CIERRO INFILE\n";

	std::ifstream infileB("/home/tfm3/workspace/Registrador/miEntradaModificada.txt");

	    //infileB >> std::setprecision(6) >> std::fixed;
		//MatrixXd B (3000,3);
		MatrixXd readingB(6000,3);
		int contLinB=0;
		//double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
		std::cout <<"antes de leer el archivo"<<std::endl;
		timestamp=0,rx=0,ry=0,rz=0,q1,q2,q3,q4=0;
		while ( infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4 ){
		        //std::cout << "You said.... contLinB"<< contLinB << timestamp << " " << rx << " " << ry << " " << rz << " " << q1 << " " << q2 << " " << q3 << " " << q4 << "\n";
		        //VectorXd myVector (rx,ry,rz);
		        //A.row(contLin)= myVector;
		        readingB.row(contLinB)<< rx,ry,rz;
		        contLinB ++;

		}
		infileB.close();
		std::cout <<"contLinB="<<contLinB<<std::endl;
		MatrixXd B (contLinB,3);
			//A = readingA.block<contLin,3>(0,0);
			B = readingB.block(0,0,contLinB,3);

	Registrador miRegistrador;
	//miRegistrador.demoPresentaMatriz();
	//MatrixXd A = MatrixXd::Random(3,3);
	//std::cout << "A "<<A<<std::endl;

	//MatrixXd B = MatrixXd::Random(3,3);
	//MatrixXd B (3000,3);
	//std::cout << "B "<<B<<std::endl;
	MatrixXd ret_R,ret_t;

	unsigned t0, t1;

	t0=clock();
	// Code to execute



	//miRegistrador.rigid_transform_3D( A, B, ret_R,ret_t);
	miRegistrador.rigid_transform_3D( B, A, ret_R,ret_t);
	t1 = clock();

	std::cout << "ret_R"<<ret_R << std::endl;
	std::cout << "ret_t "<<ret_t << std::endl;
    //--------------------------------------------- change transformation over A data, groundtruth-------------------
    /*
	MatrixXd A2 = (ret_R * A.transpose());
	//std::cout << "A2"<<A2<< std::endl;
	int N = A2.cols();
	MatrixXd A3 (A2.rows(),A2.cols());
	//std::cout << "2- "<<std::endl;
	for (int i=0; i< N; i++ ) {
		//std::cout << "3- "<<std::endl;
				//A3.row(i) = VectorXd(A2.row(i)) + VectorXd( ret_t );
		        //A3.row(i) = Vector3d( A2.row(i)) + Vector3d(ret_t);
			    A3.col(i) =  A2.col(i) + ret_t ;

		        //std::cout << "4- "<<std::endl;

			}

	//MatrixXd A3	= A2.rowwise()	+ ret_t;
	//std::cout << "A3 "<<A3<<std::endl;
	MatrixXd A4 = A3.transpose();
	//std::cout << "A4 "<<A4<<std::endl;

	// Find the error
	MatrixXd err = A4 - B;
	//std::cout << "err "<<std::endl<< err <<std::endl;
	MatrixXd err2 = err.array().pow(2);
	double errorNumber = sqrt(err2.sum()/err2.rows());

	//std::cout << "err2 "<< err2 <<std::endl;
	std::cout << "RMSE= "<< errorNumber <<std::endl;
	int numRows = A4.rows();

	for (int w=0;w<numRows; w++){
		out<<w <<" "<< A4(w,0) <<" "<< A4(w,1) <<" "<< A4(w,2) <<" "<< w <<" "<< w <<" "<<w <<" "<< w<<std::endl;
	}
    out.close();
    */


	// Apply transformation over B Data-----------------------------------------------------

	MatrixXd Final_xyz_aligned;
	Final_xyz_aligned = ret_R * B.transpose() ;
	MatrixXd mFinalTransMatrix (Final_xyz_aligned.rows(),Final_xyz_aligned.cols()),  Final_model_aligned2;
	        for (int j=0; j< Final_xyz_aligned.cols(); j++ ) {

	        	mFinalTransMatrix.col(j) << ret_t(0), ret_t(1), ret_t(2);

	        }
	        std::cout << "mFinalTransMatrix \n"<< mFinalTransMatrix <<std:: endl;
	        Final_model_aligned2=Final_xyz_aligned+mFinalTransMatrix;

	        //std::cout << "Final_model_aligned2 \n"<< Final_model_aligned2 <<std:: endl;
	        std::cout << "Final_model_aligned2.rows() \n"<< Final_model_aligned2.rows() <<std:: endl;
	        std::cout << "Final_model_aligned2.cols() \n"<< Final_model_aligned2.cols() <<std:: endl;
	        //MatrixXd alignment_error = model_aligned - data

	    int numCols = Final_model_aligned2.cols();



	    for (int w=0;w<numCols; w++){
	    	out<<w <<" "<< Final_model_aligned2(0,w) <<" "<< Final_model_aligned2(1,w) <<" "<< Final_model_aligned2(2,w) <<" "<< w <<" "<< w <<" "<< w <<" "<< w<<std::endl;
	    }
	    out.close();

	    // Find the error
	    	MatrixXd err = Final_model_aligned2 - A;
	    	//std::cout << "err "<<std::endl<< err <<std::endl;
	    	MatrixXd err2 = err.array().pow(2);
	    	double errorNumber = sqrt(err2.sum()/err2.rows());

	    	//std::cout << "err2 "<< err2 <<std::endl;
	    	std::cout << "RMSE= "<< errorNumber <<std::endl;

	    	double time = (double(t1-t0)/CLOCKS_PER_SEC);
	    	std::cout << "Execution Time: " << time << std::endl;



}
