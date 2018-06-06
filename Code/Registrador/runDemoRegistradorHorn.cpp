#include "RegistradorHorn.h"
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


	std::ifstream infile("/home/tfm3/workspace/RegistradorHorn/miEntrada.txt");
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

	std::ifstream infileB("/home/tfm3/workspace/RegistradorHorn/miEntradaModificada.txt");

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

	RegistradorHorn miRegistrador;
	//miRegistrador.demoPresentaMatriz();
	//MatrixXd A = MatrixXd::Random(3,3);
	//std::cout << "A "<<A<<std::endl;

	//MatrixXd B = MatrixXd::Random(3,3);
	//MatrixXd B (3000,3);
	//std::cout << "B "<<B<<std::endl;
	MatrixXd ret_R,ret_t,second_xyz_aligned;

	//miRegistrador.rigid_transform_3D( A, B, ret_R,ret_t);
    //miRegistrador.align(A.transpose(),B.transpose(),ret_R,ret_t);
	miRegistrador.align(B.transpose(),A.transpose(),ret_R,ret_t);
	std::cout <<"salgo de align1"<<std::endl;
    //second_xyz_aligned = ret_R * B.transpose() + ret_t;
	second_xyz_aligned = ret_R * B.transpose() ;


    MatrixXd mTransMatrix (second_xyz_aligned.rows(),second_xyz_aligned.cols()),  model_aligned2;
        for (int j=0; j< second_xyz_aligned.cols(); j++ ) {

        	mTransMatrix.col(j) << ret_t(0), ret_t(1), ret_t(2);

        }
        std::cout << "mTransMatrix \n"<< mTransMatrix <<std:: endl;
        model_aligned2=second_xyz_aligned+mTransMatrix;

        std::cout << "model_aligned2 \n"<< model_aligned2 <<std:: endl;
        //MatrixXd alignment_error = model_aligned - data

    int numCols = model_aligned2.cols();

    for (int w=0;w<numCols; w++){
    	out<<w <<" "<< model_aligned2(0,w) <<" "<< model_aligned2(1,w) <<" "<< model_aligned2(2,w) <<" "<< w <<" "<< w <<" "<< w <<" "<< w<<std:: endl;
    }
    out.close();



}
