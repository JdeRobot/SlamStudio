#include "Registrador.h"
#include <iostream>
//#include "Eigen/SVD"
//##include "Eigen/Geometry"
//#include "Eigen/Dense"
//using namespace Eigen;
using namespace std;


Registrador::Registrador(){
	std::cout<< "constructor por defecto" <<std::endl;
}//

void Registrador::rigid_transform_3D_normalized(MatrixXd A, MatrixXd B , MatrixXd& R, MatrixXd& t){
    // compare size of matrix A y B
	//Matrix A and B have been transformed . A = A - centroidA  B = B - centroidB
    int N=0;
    Matrix3d AA, BB , mCentroidA, mCentroidB;
    MatrixXd H,  U , V ,S;

    std::cout<<"rigid_transform_3D 1"<<std::endl;
    if (A.cols()*A.rows() == B.cols()* B.rows()) { // the 2 arrays have similar size
    	MatrixXd AA(A.rows(),A.cols()), BB (B.rows(),B.cols()) , mCentroidA(A.rows(),A.cols()), mCentroidB(B.rows(),B.cols());

		N = A.rows(); // total points


		H= A.transpose() * B;



		//std::cout << "H \n"<< H << std::endl;

		//JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
		JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
		//BDCSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
		U=svd.matrixU();
		V=svd.matrixV();
		S=svd.singularValues();
		//MatrixXd neoA = U*S.asDiagonal()*V.transpose();
		//MatrixXd neoA = neoA * V;
				//*V.transpose();
		//MatrixXd neoA = U*S*V.transpose();
		//std::cout << "neoA: \n" << neoA << std::endl;
		MatrixXd Vt = V.transpose();
		//std::cout << "U: \n" << U << std::endl;;
		//std::cout << "V: \n" << V << std::endl;;
		//std::cout << "Vt: \n" << Vt << std::endl;
		//std::cout << "S: \n" << S << std::endl;

		//R = V.transpose() * U.transpose();
		R = Vt.transpose() * U.transpose();
		std::cout << "R1: \n" << R <<std::endl;

		if (R.determinant() < 0){
			std::cout << "Detectadas reflections-------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>><"<<std::endl;
		       //Vt[2,:] *= -1
			    Vt.row(2) *= -1;
			    R = Vt.transpose() * U.transpose();
			    std::cout << "R2: \n" << R <<std::endl;
		}
		//t = -R * mCentroidA.transpose() + mCentroidB.transpose();
		//t = -R * centroidA.transpose() + centroidB.transpose();
		//t = -R * centroidA.transpose();
		Vector3d centroidA = A.colwise().mean();
		Vector3d centroidB = B.colwise().mean();

        //t = -R * centroidA + centroidB;
        t = R * centroidB - centroidA;
		std::cout <<"t \n"<< t <<std:: endl;

		//return R, t

		//Hallar los centroides de A y  B
		//centroid_A = mean(A, axis=0)
		//centroid_B = mean(B, axis=0)

		// Diferencia de los centroides
		//AA = A - tile(centroid_A, (N, 1))
		//BB = B - tile(centroid_B, (N, 1))

		//print "AA shape";
		//print  AA.shape;
		//print BB.shape;

		//# dot is matrix multiplication for array

		//H = transpose(AA)* BB

		//Hallar SVD
		//U, S, Vt = linalg.svd(H)

		//R = Vt.T * U.T

		//# special reflection case
		//if linalg.det(R) < 0:
		  // print "Reflection detected"
		  //Vt[2,:] *= -1
		  // R = Vt.T * U.T

		//t = -R*centroid_A.T + centroid_B.T

		//print t

		//return R, t
    }
}


 void Registrador::rigid_transform_3D(MatrixXd A, MatrixXd B , MatrixXd& R, MatrixXd& t){

    int N=0;
    Matrix3d AA, BB , mCentroidA, mCentroidB;
    MatrixXd H,  U , V ,S;

    std::cout<<"rigid_transform_3D 1"<<std::endl;
    if (A.cols()*A.rows() == B.cols()* B.rows()) { // the 2 arrays have similar size
    	MatrixXd AA(A.rows(),A.cols()), BB (B.rows(),B.cols()) , mCentroidA(A.rows(),A.cols()), mCentroidB(B.rows(),B.cols());

		N = A.rows(); // total points

		double centroidAX= VectorXd(A.col(0)).mean();
		double centroidAY= VectorXd(A.col(1)).mean();
		double centroidAZ= VectorXd(A.col(2)).mean();

		Vector3d centroidA(centroidAX,centroidAY,centroidAZ);

		double centroidBX= VectorXd(B.col(0)).mean();
		double centroidBY= VectorXd(B.col(1)).mean();
		double centroidBZ= VectorXd(B.col(2)).mean();

		Vector3d centroidB(centroidBX,centroidBY,centroidBZ);

		for (int i=0; i< N; i++ ) {
			//AA.row(i) = Vector3d(A.row(i)) - centroidA ;
			mCentroidA.row(i) << centroidAX ,centroidAY,centroidAZ ;

		}

		for (int j=0; j< N; j++ ) {
			//BB.row(j) = Vector3d(B.row(j)) - centroidB ;
			mCentroidB.row(j) << centroidBX ,centroidBY,centroidBZ ;

		}
        AA= A - mCentroidA;
        BB= B - mCentroidB;
		//std::cout << "A \n"<< A <<std:: endl;

		std::cout <<centroidA <<std::endl;
		//std::cout << "AA \n"<< AA << std::endl;

		//std::cout << "B \n"<< B << std::endl;

		std::cout <<centroidB <<std::endl;
		//std::cout << "BB \n"<< BB << std::endl;

		H= AA.transpose() * BB;



		//std::cout << "H \n"<< H << std::endl;

		//JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
		JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
		//BDCSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
		U=svd.matrixU();
		V=svd.matrixV();
		S=svd.singularValues();
		//MatrixXd neoA = U*S.asDiagonal()*V.transpose();
		//MatrixXd neoA = neoA * V;
				//*V.transpose();
		//MatrixXd neoA = U*S*V.transpose();
		//std::cout << "neoA: \n" << neoA << std::endl;
		MatrixXd Vt = V.transpose();
		//std::cout << "U: \n" << U << std::endl;;
		//std::cout << "V: \n" << V << std::endl;;
		//std::cout << "Vt: \n" << Vt << std::endl;
		//std::cout << "S: \n" << S << std::endl;

		//R = V.transpose() * U.transpose();
		R = Vt.transpose() * U.transpose();
		std::cout << "R1: \n" << R <<std::endl;
		std::cout << "centroidA: " << centroidA <<std::endl;
		std::cout << "centroidB: " << centroidB <<std::endl;
		if (R.determinant() < 0){
            std::cout << "Detected reflections-------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>><"<<std::endl;
		       //Vt[2,:] *= -1
			    Vt.row(2) *= -1;
			    R = Vt.transpose() * U.transpose();
			    std::cout << "R2: \n" << R <<std::endl;
		}
		//t = -R * mCentroidA.transpose() + mCentroidB.transpose();
		//t = -R * centroidA.transpose() + centroidB.transpose();
		//t = -R * centroidA.transpose();
        //t = -R * centroidA + centroidB;
        //t= -R * centroidB + centroidA;
        t= -R*centroidA + centroidB;
		std::cout <<"t \n"<< t <<std:: endl;

		//return R, t

		//Hallar los centroides de A y  B
		//centroid_A = mean(A, axis=0)
		//centroid_B = mean(B, axis=0)

		// Diferencia de los centroides
		//AA = A - tile(centroid_A, (N, 1))
		//BB = B - tile(centroid_B, (N, 1))

		//print "AA shape";
		//print  AA.shape;
		//print BB.shape;

		//# dot is matrix multiplication for array

		//H = transpose(AA)* BB

		//Hallar SVD
		//U, S, Vt = linalg.svd(H)

		//R = Vt.T * U.T

		//# special reflection case
		//if linalg.det(R) < 0:
		  // print "Reflection detected"
		  //Vt[2,:] *= -1
		  // R = Vt.T * U.T

		//t = -R*centroid_A.T + centroid_B.T

		//print t

		//return R, t
    }
}

 void Registrador::applyTransformationsOverData(MatrixXd dataInitial,MatrixXd& dataEstimated, MatrixXd rotationEstimated, MatrixXd traslationEstimated){
         MatrixXd Final_xyz_aligned;
         Final_xyz_aligned = rotationEstimated * dataInitial.transpose() ;
         //Final_xyz_aligned = rotationEstimated.transpose() * dataInitial.transpose() ;
         MatrixXd mFinalTransMatrix (Final_xyz_aligned.rows(),Final_xyz_aligned.cols()),  Final_model_aligned2;
                 for (int j=0; j< Final_xyz_aligned.cols(); j++ ) {

                     mFinalTransMatrix.col(j) << traslationEstimated(0), traslationEstimated(1), traslationEstimated(2);

                 }
                 //std::cout << "mFinalTransMatrix \n"<< mFinalTransMatrix <<std:: endl;
                 Final_model_aligned2=Final_xyz_aligned+mFinalTransMatrix;
                 //Final_model_aligned2=Final_xyz_aligned-mFinalTransMatrix;

                 //std::cout << "Final_model_aligned2 \n"<< Final_model_aligned2 <<std:: endl;
                 //std::cout << "Final_model_aligned2.rows() \n"<< Final_model_aligned2.rows() <<std:: endl;
                 //std::cout << "Final_model_aligned2.cols() \n"<< Final_model_aligned2.cols() <<std:: endl;
                 //MatrixXd alignment_error = model_aligned - data

             int numCols = Final_model_aligned2.cols();

         //the Estimated data will return on this MatrixXd
         dataEstimated=Final_model_aligned2.transpose();

             // Save estimated data to a file
             /*
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
                 std::cout << "ExecutdataEstimatedQuaternionion Time: " << time << std::endl;
             */
 }

 void Registrador::applyTransformationsOverQuaternion (MatrixXd dataInitialQuaternion,MatrixXd& dataEstimatedQuaternion, MatrixXd rotMatrix) {
     // dataInitialQuaternion will have q1,q2,q3,q4
     // dataEstimated will have q1',q2',q3',q4' estimated
     // rotMatrix is rotationMatrixEstimated

     Matrix3d matRot_toQuaternion; //important rotMatrix should be 3x3 to convert to a quaternion
     matRot_toQuaternion<< rotMatrix(0,0),rotMatrix(0,1),rotMatrix(0,2),
                           rotMatrix(1,0),rotMatrix(1,1) ,rotMatrix(1,2),
                           rotMatrix(2,0),rotMatrix(2,1),rotMatrix(2,2);

     Quaterniond q(matRot_toQuaternion);

     q.normalize(); //important quaternion from rotation matrix should be normalized
     dataEstimatedQuaternion= MatrixXd::Zero(dataInitialQuaternion.rows(),dataInitialQuaternion.cols());
     int rows = dataInitialQuaternion.rows();
     for (int i=0;i< rows;i++){
         //create quaternion p (p1 p2 p3 p4)
         Quaterniond pData(dataInitialQuaternion(i,0),dataInitialQuaternion(i,1),dataInitialQuaternion(i,2),dataInitialQuaternion(i,3));
         Quaterniond rotatedP= q*pData*q.inverse();
         dataEstimatedQuaternion(i,0)= rotatedP.w();
         dataEstimatedQuaternion(i,1)= rotatedP.x();
         dataEstimatedQuaternion(i,2)= rotatedP.y();
         dataEstimatedQuaternion(i,3)= rotatedP.z();



     }



 }

void Registrador::demoPresentaMatriz() {

  MatrixXd m = MatrixXd::Random(3,3);
  //m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
  std::cout << "m =" <<  m <<std::endl;
  VectorXd v(3);
  v << 1, 2, 3;
  std::cout << "m * v =" <<  m * v <<std::endl;

}

