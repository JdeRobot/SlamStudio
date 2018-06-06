#include "RegistradorHorn.h"
#include <iostream>
#include "Eigen/SVD"
//#include "Eigen/Dense"
//using namespace Eigen;
//using namespace std;

RegistradorHorn::RegistradorHorn(){
	std::cout<< "constructor por defecto" <<std::endl;
}//

void RegistradorHorn::demoPresentaMatriz() {

  MatrixXd m = MatrixXd::Random(3,3);
  //m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
  std::cout << "m =" <<  m <<std::endl;
  VectorXd v(3);
  v << 1, 2, 3;
  std::cout << "m * v =" <<  m * v <<std::endl;

}

void RegistradorHorn::align(MatrixXd model,MatrixXd data, MatrixXd& rot, MatrixXd& trans){
	std::cout << "model \n" <<model<<std:: endl;
	std::cout << "data \n" <<data<<std:: endl;
	std::cout << "RegistradorHorn1 \n" <<std:: endl;
	MatrixXd model_zerocentered(model.rows(),model.cols()), data_zerocentered(data.rows(),data.cols()) , mCentroidModel(model.rows(),model.cols()), mCentroidData(data.rows(),data.cols());
	std::cout << "RegistradorHorn2 \n" <<std:: endl;
	MatrixXd U,Vh;

			int N = model.cols(); // total points

			double centroidModelX= VectorXd(model.row(0)).mean();
			double centroidModelY= VectorXd(model.row(1)).mean();
			double centroidModelZ= VectorXd(model.row(2)).mean();

			Vector3d centroidModel(centroidModelX,centroidModelY,centroidModelZ);

			double centroidDataX= VectorXd(data.row(0)).mean();
			double centroidDataY= VectorXd(data.row(1)).mean();
			double centroidDataZ= VectorXd(data.row(2)).mean();

			Vector3d centroidData(centroidDataX,centroidDataY,centroidDataZ);
			std::cout << "RegistradorHorn3 \n" <<std:: endl;
			std::cout << "centroidModelX \n" <<centroidModelX<<std:: endl;
			std::cout << "centroidModelY \n" <<centroidModelY<<std:: endl;
			std::cout << "centroidModelZ \n" <<centroidModelZ<<std:: endl;
			std::cout << "N  \n" <<N <<std:: endl;
			for (int i=0; i< N; i++ ) {
				//AA.row(i) = Vector3d(A.row(i)) - centroidA ;
				//mCentroidModel.col(i) << centroidModelX , centroidModelY, centroidModelZ ;
				mCentroidModel.col(i) = Vector3d( centroidModelX , centroidModelY, centroidModelZ ) ;
				//std::cout << "mCentroidModel.col(i)\n" << mCentroidModel.col(i) << std:: endl;
			}

			//std::cout << "mCentroidModel \n"<< mCentroidModel <<std:: endl;

			for (int j=0; j< N; j++ ) {
				//BB.row(j) = Vector3d(B.row(j)) - centroidB ;
				mCentroidData.col(j) << Vector3d (centroidDataX ,centroidDataY,centroidDataZ );
				//std::cout << "mCentroidData.col(j)\n" << mCentroidData.col(j) << std:: endl;

			}

			//std::cout << "mCentroidData \n"<< mCentroidData <<std:: endl;

			std::cout << "RegistradorHorn4 \n" <<std:: endl;
			model_zerocentered = model - mCentroidModel;
			std::cout << "model_zerocentered \n"<< model_zerocentered <<std:: endl;
			data_zerocentered = data - mCentroidData;
			std::cout << "data_zerocentered \n"<< data_zerocentered <<std:: endl;
	//model_zerocentered = model -model.mean(1);
	//data_zerocentered = data -data.mean(1);

	Matrix3d W= Matrix3d::Zero();
	Matrix3d a;
	std::cout << "RegistradorHorn5 \n" <<std:: endl;
	for (int column=0;  column < model.cols();column ++) {
		std::cout << "RegistradorHorn6 \n" <<std:: endl;
		a=Vector3d(model_zerocentered.col(column))*Vector3d(data_zerocentered.col(column)).transpose();
		std::cout << "a \n"<< a <<std:: endl;
		W=W+a.transpose();
		//std::cout << "W \n"<< W <<std:: endl;
	}
	std::cout << "W.transpose() \n"<< W.transpose() <<std:: endl;
	JacobiSVD<MatrixXd> svd(W, ComputeFullU | ComputeFullV);
	//JacobiSVD<MatrixXd> svd(W.transpose(), ComputeFullU | ComputeFullV);

			//BDCSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
	U=svd.matrixU();
	std::cout << "U \n"<< U <<std:: endl;
	Vh=svd.matrixV();
	std::cout << "Vh \n"<< Vh <<std:: endl;
	//S=svd.singularValues();


	Matrix3d S;
    S << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    if (U.determinant() * Vh.determinant() < 0){
    	std::cout << "Producto negativo: cambio S(2,2) a -1 \n"<< std:: endl;
    	S(2,2)= -1;

    }
    rot = U*S*Vh.transpose();
    std::cout << "rot \n"<< rot <<std:: endl;
    //trans = mCentroidData - (rot * mCentroidModel);
    //trans = centroidData - rot * centroidModel;
    //t = -R * centroidA + centroidB;
    trans = -rot * centroidModel + centroidData;
    //trans = -rot * centroidData + centroidModel;

    std::cout << "trans \n"<< trans <<std:: endl;
    //trans = data.mean() -rot * model.mean()

    //MatrixXd model_aligned = rot * model + trans;
    //MatrixXd model_aligned = trans + rot * model ;

    MatrixXd model_aligned = rot * model ;
    MatrixXd mTransMatrix (model_aligned.rows(),model_aligned.cols()),model_aligned2;
    for (int j=0; j< model_aligned.cols(); j++ ) {

    	mTransMatrix.col(j) << trans(0), trans(1), trans(2);

    }
    std::cout << "mTransMatrix \n"<< mTransMatrix <<std:: endl;
    model_aligned2=model_aligned+mTransMatrix;

    std::cout << "model_aligned2 \n"<< model_aligned2 <<std:: endl;
    //MatrixXd alignment_error = model_aligned - data

    //trans_error =
}


void RegistradorHorn::rigid_transform_3D (MatrixXd A, MatrixXd B, MatrixXd& R, MatrixXd& t){

	std::cout << "inside rigid_transform_3D" << std::endl;
}
