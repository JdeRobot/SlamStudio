

#include "GeneratorPCA.h"

using namespace Eigen;


GeneratorPCA::GeneratorPCA(){
	std::cout<< "constructor por defecto AjusteTiempo" <<std::endl;
}
void GeneratorPCA::rotatePCA (int cont,MatrixXd& PCA){
	// This function rotates the 3 principal components estimated by pca
	// there are 6 possibilities, 3!= 3*2, therefore 5 possible rotations
	//  cont = 0   XYZ
	//  cont = 1   XZY
	//  cont = 2   YZX
	//  cont = 3   YXZ
	//  cont = 4   ZXY
	//  cont = 5   ZYX

	MatrixXd aux= PCA.block(0,0,3,3);
	//if (cont==1 || cont == 3){
	// if cont == 0 the order does not change
	if (cont==1 ){
        // XYZ -->  XZY
		aux.col(1)= PCA.col(1);
		PCA.col(1)= PCA.col(2);
		PCA.col(2)= aux.col(1);


	} else if (cont==2 ){
        // XYZ --> YZX
		aux.col(0)=PCA.col(0);
		PCA.col(0)=PCA.col(1);
		PCA.col(1)=PCA.col(2);
		PCA.col(2)=aux.col(0);

	}else if (cont==3){
		// XYZ --> YXZ

		aux.col(0)=PCA.col(0);
		PCA.col(0)=PCA.col(1);
		PCA.col(1)=aux.col(0);

	}else if (cont==4){
		// XYZ --> ZXY
		aux.col(0)=PCA.col(0);
		PCA.col(0)=PCA.col(2);
		PCA.col(2)=PCA.col(1);
		PCA.col(1)=aux.col(0);

	}else if (cont==5){
		// XYZ --> ZYX
		aux.col(0)=PCA.col(0);
		PCA.col(0)=PCA.col(2);
		PCA.col(2)=aux.col(0);

	}
}
void GeneratorPCA::calculatePCAbySVD(int maxLine, MatrixXd& A){
	/*
	 * based on code
	 * https://forum.kde.org/viewtopic.php?f=9&t=110265
	 */
	//READS A FILE WITH THE DATA

	std::cout << std::setprecision(6) << std::fixed;
		std::ifstream infile( "/home/tfm3/workspace/GeneratorPCA/original_data.txt" );
		//inputFile >> std::setprecision(6) >> std::fixed;

	    std::cout << std::setprecision(6) << std::fixed;
	    std::ofstream outB( "/home/tfm3/workspace/GeneratorPCA/miSalidaB.txt" );
	    outB << std::setprecision(6) << std::fixed;


		std::cout <<"antes de declarar la variable"<<std::endl;
		//int maxLine = 30;
		MatrixXd readingA (maxLine,3);//size is big enough to store an unknown number of data rows from file

		VectorXd vTiempoA (maxLine), vSerieA (maxLine) ;
		//read a file
		int contLin=0;
		double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
		double mediaTiempoA, mediaTiempoB, mediaSerieA, mediaSerieB;
		std::cout <<"antes de leer el archivo"<<std::endl;
		//int miOffset=5;
	    //double miOffset = offset;
	    double newContLin = 0;
	    //std::cout <<"miOffset="<<miOffset<<std::endl;

	    //Read the input file
		while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
			    readingA.row(contLin)<< rx,ry,rz;

				contLin ++;

		}
		infile.close();

		// Calculate PCA ,


	A = readingA.block(0,0,contLin,3);
	Eigen::MatrixXd aligned = A.rowwise() - A.colwise().mean();
	Eigen::MatrixXd cov = aligned.adjoint() * aligned;
		cov = cov / (A.rows() - 1);
	//aligned = aligned / (aligned.rows() - 1);

	// we can directly take SVD
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(aligned, Eigen::ComputeThinV);
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeThinV);

	// and here is the question what is the basis matrix and how can i reduce it
	// in my understanding it should be:
	Eigen::MatrixXd pcaTransform = svd.matrixV().leftCols(3);
	//Eigen::MatrixXd pcaTransform = svd.matrixV().rightCols(3);

	// then to project the data:
	A=pcaTransform.block(0,0,3,3);
	//IMPORTANT : change order of columns . Column 0 by colum 2. When calculate PCA normally returns cols in increasing value.
	//When calculate PCA using SVD columns are returned in decreasing value.
	MatrixXd mAux= pcaTransform.block(0,0,3,3);
	mAux.col(0)=A.col(0);
	A.col(0)=A.col(2);
	A.col(2)=mAux.col(0);
	std::cout <<"svd.matrixV_A="<<A<<std::endl;
	pcaTransform= A.block(0,0,3,3);
	Eigen::MatrixXd projected = aligned * pcaTransform;

	for (int i= 0; i< contLin; i++){
			//std::cout <<"i="<<i<<std::endl;
			//std::cout <<"projected.row(i)="<<projected.row(i)<<std::endl;
			VectorXd aRow = projected.row(i);
			outB << aRow(0)<<" "<<aRow(1)<<" "<<aRow(2)<<"\n";
			//std::cout <<"aRow="<<aRow<<std::endl;
			//outA << centered.row(i).[1], centered.row(i).[2],centered.row(i).[3];
			//outA << centered.row(i,1), centered.row(i,2),centered.row(i,3);

		}

	outB.close();

	std::cout <<"pcaTransform SVD="<<pcaTransform<<std::endl;
	std::cout <<"pcaTransformRight="<<svd.matrixV().rightCols(3)<<std::endl;
	std::cout <<"pcaTransformLeft="<<svd.matrixV().leftCols(3)<<std::endl;
	std::cout <<"svd.matrixV="<<svd.matrixV()<<std::endl;
	//std::cout <<"svd.matrixU="<<svd.matrixU()<<std::endl;


}

void GeneratorPCA::setPcaA(MatrixXd aMatrix){
    pcaA=aMatrix;
}
void GeneratorPCA::setPcaB(MatrixXd aMatrix){
    pcaB=aMatrix;
}

MatrixXd GeneratorPCA::getPcaB(){
   return pcaB;
}
MatrixXd GeneratorPCA::getPcaA(){
   return pcaA;
}


void GeneratorPCA::calculatePCAbySVD( int rotation,MatrixXd& A , MatrixXd& A2 ,MatrixXd& PCA){
	/*
	 * based on code
	 * https://forum.kde.org/viewtopic.php?f=9&t=110265
	 */
    // int rotation , means one of the six possibles orders of pca solution. XYZ, XZY, YZX,YXZ,ZXY,ZYX

	Eigen::MatrixXd aligned = A.rowwise() - A.colwise().mean();
	Eigen::MatrixXd cov = aligned.adjoint() * aligned;
	cov = cov / (A.rows() - 1);
	//aligned = aligned / (aligned.rows() - 1);

	// we can directly take SVD
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(aligned, Eigen::ComputeThinV);
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeThinV);

	// and here is the question what is the basis matrix and how can i reduce it
	// in my understanding it should be:
	Eigen::MatrixXd pcaTransform = svd.matrixV().leftCols(3);
    PCA=pcaTransform.block(0,0,3,3);
	//Eigen::MatrixXd pcaTransform = svd.matrixV().rightCols(3);

	// then to project the data:
	MatrixXd Aux1=pcaTransform.block(0,0,3,3);
	//IMPORTANT : change order of columns . Column 0 by colum 2. When calculate PCA normally returns cols in increasing value.
	//When calculate PCA using SVD columns are returned in decreasing value.

	rotatePCA(rotation,Aux1);
	//std::cout <<"svd.matrixV_A1="<<Aux1<<std::endl;
	//std::cout <<"antesDeTerminarcalculatePCAbySVD1"<<std::endl;
	pcaTransform= Aux1.block(0,0,3,3);
	//std::cout <<"antesDeTerminarcalculatePCAbySVD1"<<std::endl;
	//std::cout <<"svd.matrixV_pcaTransform="<<pcaTransform<<std::endl;
	Eigen::MatrixXd projected = aligned * pcaTransform;
	//std::cout <<"antesDeTerminarcalculatePCAbySVD"<<std::endl;



	//std::cout <<"pcaTransform SVD="<<pcaTransform<<std::endl;
	//std::cout <<"pcaTransformRight="<<svd.matrixV().rightCols(3)<<std::endl;
	//std::cout <<"pcaTransformLeft="<<svd.matrixV().leftCols(3)<<std::endl;
	//std::cout <<"svd.matrixV="<<svd.matrixV()<<std::endl;
	//std::cout <<"svd.matrixU="<<svd.matrixU()<<std::endl;
	// Return the dataset transformed by pca
    A2=projected.block(0,0,projected.rows(),projected.cols());
    //std::cout <<"A2="<<A2<<std::endl;

}

void GeneratorPCA::calculatePCA(int maxLine, MatrixXd& A){


	std::cout << std::setprecision(6) << std::fixed;
	std::ifstream infile( "/home/tfm3/workspace/GeneratorPCA/original_data.txt" );
	//inputFile >> std::setprecision(6) >> std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/GeneratorPCA/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;


	std::cout <<"antes de declarar la variable"<<std::endl;
	//int maxLine = 30;
	MatrixXd readingA (maxLine,3);//size is big enough to store an unknown number of data rows from file

	VectorXd vTiempoA (maxLine), vSerieA (maxLine) ;
	//read a file
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	double mediaTiempoA, mediaTiempoB, mediaSerieA, mediaSerieB;
	std::cout <<"antes de leer el archivo"<<std::endl;
	//int miOffset=5;
    //double miOffset = offset;
    double newContLin = 0;
    //std::cout <<"miOffset="<<miOffset<<std::endl;

    //Read the input file
	while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
		    readingA.row(contLin)<< rx,ry,rz;

			contLin ++;

	}
	infile.close();

	// Calculate PCA ,
	//MatrixXd A (contLin,3);

	A = readingA.block(0,0,contLin,3);

	// Mean centering data.
	//Eigen::VectorXd myMeans = A.colwise().mean();

	//std::cout <<"myMeans="<<myMeans<<std::endl;

   //Eigen::MatrixXd centered = A.rowwise() + myMeans.transpose();
   //Eigen::MatrixXd centered = A.rowwise() - myMeans.transpose();
   Eigen::MatrixXd centered = A.rowwise()- A.colwise().mean();

    	// Compute the covariance matrix.
	Eigen::MatrixXd cov = centered.adjoint() * centered;
	cov = cov / (A.rows() - 1);

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	// Normalize eigenvalues to make them represent percentages.

	Eigen::VectorXd normalizedEigenValues =  eig.eigenvalues() / eig.eigenvalues().sum();


	// Get the 3 major eigenvectors and omit the others.
	Eigen::MatrixXd evecs = eig.eigenvectors();
	Eigen::MatrixXd pcaTransform = evecs.rightCols(3);
	std::cout <<"evecs="<<evecs<<std::endl;


	// Map the dataset in the new three dimensional space.
	centered = centered * pcaTransform;


    outA.close();

    std::cout <<"pcaTransform PCA="<<pcaTransform<<std::endl;



}



