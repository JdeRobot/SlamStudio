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
#include "Interpolator.h"
//#include "../AjusteTiempo/AjusteTiempo.h"
#include "AjusteTiempo.h"


/* This class interpolates 2 data of series
 * Reads 2 files with format " Time X Y Z"
 *
 */


Interpolator::Interpolator(){

	std::cout<< "constructor por defecto Interpolator" <<std::endl;

}



void Interpolator::insertRowInSequence (MatrixXd& m, VectorXd myVector, int position){
   /*
	MatrixXd myNewMatrix ( m.rows()+1,m.cols());
	std::cout <<"myNewMatrix="<<myNewMatrix<<std::endl;
	std::cout<< "insertRowInSequence"<<std::endl;
	std::cout <<"m="<<m<<std::endl;


	//myNewMatrix << m.block(position-1,m.cols(),0,0);
	//myNewMatrix = m.block(0,0,position-1,3);
	std::cout <<"myNewMatrix="<<myNewMatrix<<std::endl;
	//myNewMatrix.row(position) << myVector(0),myVector(1),myVector(2);
	//myNewMatrix.row(position) = myVector.transpose();

	//myNewMatrix << m.block(position,m.cols(),position,0);
	//myNewMatrix << m.block(0,0,position,3);

	//return myNewMatrix;

	for (int i=0; i< m.rows(); i++){
		if (i<position){
			std::cout <<"1"<<std::endl;
			myNewMatrix.row(i)<<m.row(i);
		} else if (i>position){
			std::cout <<"2"<<std::endl;
			myNewMatrix.row(i+1) <<m.row(i);
		} else if (i==position){
			std::cout <<"3"<<std::endl;
			myNewMatrix.row(position) << myVector.transpose();
			myNewMatrix.row(i+1) << m.row(i);
		}
		std::cout <<"myNewMatrix="<<myNewMatrix<<std::endl;
	}
	std::cout<< "insertRowInSequence2"<<std::endl;
	m=myNewMatrix.block(0,0,myNewMatrix.rows(),4);
	//return;
	*/
}




double Interpolator::interpolateValue (double x,double x2,double y2,double x3,double y3) {

	double y;
	y= y2 + (x-x2)*((y3-y2)/(x3-x2));

	return y;
}






MatrixXd Interpolator::interpolateAoverB(MatrixXd& A, MatrixXd& B) {


	double valueA=0,antB=0,sigB=0;
	int contA=0,contB = 0, newContB=0;
	valueA=A.row(0)(0);
	contA++;
	antB=B.row(0)(0);
	contB++;
	sigB=B.row(contB)(0);
	//contB++;

	MatrixXd newBMatrix (A.rows()+B.rows(),A.cols());
	newBMatrix.row(newContB)<< B.row(0);
	newContB++;
	while (contA < A.rows() && contB < B.rows()){
		std::cout<< "1-interpolateAoverB "<<"valueA="<<valueA<<" antB="<<antB<<" sigB="<<sigB<<" newBMatrix.row(newContB)(0)="<< newBMatrix.row(newContB)(0)<<"contA= "<<contA<<"contB= "<<contB<<"A.rows()="<<A.rows()<<"B.rows()="<<B.rows()<<std::endl;
		if ((valueA > antB) & (valueA < sigB)) {
			while ((valueA > antB )&& (valueA < sigB )&&( contA < A.rows())) {
				std::cout<< "interpolo1 "<<std::endl;
				double y2= (B.row(contB-1))(1);
				double x = (A.row(contA-1))(0);
				double x2= (B.row(contB-1))(0);
				double y3= (B.row(contB))(1);
				double x3= (B.row(contB))(0);
				double xCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo2 "<<std::endl;
				//std::cout<< "interpolo X"<<std::endl;
				//interpolate Y coordinate
				y2= (B.row(contB-1))(2);
				x = (A.row(contA-1))(0);
				x2= (B.row(contB-1))(0);
				y3= (B.row(contB))(2);
				x3= (B.row(contB))(0);
				double yCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo3 "<<std::endl;
				//std::cout<< "interpolo Y"<<std::endl;
				//interpolate Z coordinate
				y2= (B.row(contB-1))(3);
				x = (A.row(contA-1))(0);
				x2= (B.row(contB-1))(0);
				y3= (B.row(contB))(3);
				x3= (B.row(contB))(0);
				double zCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo4 "<<std::endl;
				//std::cout<< "interpolo X,y,Z"<<std::endl;
				Vector4d myNewVector(x,xCoordinate,yCoordinate,zCoordinate);
				//std::cout<< "myNewVector="<<myNewVector<<std::endl;
				//contAddedValues++;

				newBMatrix.row(newContB)<< myNewVector.transpose();
				std::cout<< "interpolo1 "<<"newBMatrix.row(newContB)="<<newBMatrix.row(newContB)<<std::endl;
				newContB ++;
				if (contA < A.rows()){
					valueA=A.row(contA)(0);
				}
				contA++;
				//interpolated = true;
				std::cout<< "interpoloFIN "<<std::endl;
			}

		} else if (valueA <= antB) {
			std::cout<< "valueA <= antB "<<std::endl;
			if (contA < A.rows()){
				valueA=A.row(contA)(0);
			}
			contA++;
		} else if (valueA == sigB){
			std::cout<< "valueA == sigB"<<std::endl;
			//if ((newContB>=2) && newBMatrix.row(newContB-1)(0) > newBMatrix.row(newContB-2)(0)) {// to avoid inserting duplicates rows
				newBMatrix.row(newContB)<< B.row(contB);
				newContB ++;
			//}
			contB++;
			if (contB < B.rows()) {
				antB=sigB;
				sigB=B.row(contB)(0);
			}
			/*if (contA < A.rows()){
				valueA=A.row(contA)(0);
			}
			contA++;*/


		} else if (valueA > sigB){
			std::cout<< "valueA > sigB"<<std::endl;
			//if ((newContB>=2) && newBMatrix.row(newContB-1)(0) > newBMatrix.row(newContB-2)(0)) {// to avoid inserting duplicates rows
				newBMatrix.row(newContB)<< B.row(contB);
				newContB ++;
			//}
			contB++;
			if (contB < B.rows()) {
				antB=sigB;
				sigB=B.row(contB)(0);
			}

		}
		std::cout<<" 2-interpolateAoverB "<<"valueA="<<valueA<<" antB="<<antB<<" sigB="<<sigB<<" newBMatrix.row(newContB)(0)="<< newBMatrix.row(newContB)(0)<<"contA= "<<contA<<"contB= "<<contB<<"A.rows()="<<A.rows()<<"B.rows()="<<B.rows()<<std::endl;

	if ((newContB>=2) && (newBMatrix.row(newContB-2)(0) == newBMatrix.row(newContB-1)(0)))	{

		std::cout<< newBMatrix.row(newContB-2)(0)<<" = " << newBMatrix.row(newContB-1)(0)<<"ENCONTRADO DUPLICADO>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<std::endl;
		exit(0);

	}

	}//end while
	std::cout<< "end While="<<std::endl;
	std::cout<< "interpolateAoverB: end While "<<"contA="<<contA<<"contB="<<contB<<"A.rows()="<<A.rows()<<"B.rows()="<<B.rows()<<std::endl;
	while (contB < B.rows()){
		newBMatrix.row(newContB)<< B.row(contB);
		newContB ++;
		contB++;
	}
	return  newBMatrix;
}



void Interpolator::interpolateSerieToFrequency(float frequency, MatrixXd& B){

	/*
	 * This method interpolates a matrix with a given frequency
	 * For instance: Matrix 1,2,3,4,5
	 * Given frequency: 05
	 * Result of interpolation : Matrix 1, 1.5 , 2 , 2.5, 3 , 3.5 , 4 , 4.5 , 5
	 */

	double AntValueB=0,ActValueA=0,ActValueB=0;
	int contA=0,contB = 0, newContB=0;

	AntValueB=B.row(0)(0);
	//ActValueA += frequency;
	ActValueB=B.row(1)(0);
	ActValueA = ActValueB;//the frequency starts with the same value that the first element of the serie B. Later it will be increased by frequency

	contB=1;

	double newValue=0;
	MatrixXd newBMatrix (3*B.rows(),B.cols());
	newBMatrix.row(newContB)<< B.row(0);
	newContB++;
	int interpolated=0,endInterpolate = 0;
	long actB,actA,antB;
	//while ( (contB) <= B.rows() && (contA ) < A.rows()){// begin while 1
	while ( !endInterpolate &&  contB < B.rows()){
		interpolated = 0;
		std::cout<< "1 ActValueA="<<ActValueA<<"ActValueB="<<ActValueB<<"AntValueB="<<AntValueB<<std::endl;
        //while ((float(ActValueA)< float(ActValueB)) && (float(ActValueA) > float(AntValueB))  ) { //begin while 2
		//while (((ActValueB - ActValueA) > 0 ) && ((ActValueA - AntValueB) >0)  ) { //begin while 2

		actB = ActValueB*10000;
		actA = ActValueA*10000;
		antB = AntValueB*10000;
		std::cout<< "1 actB="<<actB<<"actA="<<actA<<"antB="<<antB<<std::endl;
		while (actA < actB && actA > antB ) { //begin while 2
			std::cout<< "2 ActValueA="<<ActValueA<<"ActValueB="<<ActValueB<<"AntValueB="<<AntValueB<<std::endl;
			double y2= (B.row(contB-1))(1);
			double x = (ActValueA);
			double x2= (B.row(contB-1))(0);
			double y3= (B.row(contB))(1);
			double x3= (B.row(contB))(0);
			double xCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
			//std::cout<< "interpolo X"<<std::endl;
			//interpolate Y coordinate
			y2= (B.row(contB-1))(2);
			x = (ActValueA);
			x2= (B.row(contB-1))(0);
			y3= (B.row(contB))(2);
			x3= (B.row(contB))(0);
			double yCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
			//std::cout<< "interpolo Y"<<std::endl;
			//interpolate Z coordinate
			y2= (B.row(contB-1))(3);
			x = (ActValueA);
			x2= (B.row(contB-1))(0);
			y3= (B.row(contB))(3);
			x3= (B.row(contB))(0);
			double zCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
			//std::cout<< "interpolo X,y,Z"<<std::endl;
			Vector4d myNewVector(x,xCoordinate,yCoordinate,zCoordinate);

			std::cout<< "myNewVector="<<myNewVector.transpose()<<std::endl;

			//contAddedValues++;
			newBMatrix.row(newContB)<< myNewVector.transpose();
			newContB ++;

			ActValueA += frequency;

			interpolated = true;
			//actB = ActValueB*10000;
			actA = ActValueA*10000;
			//antB = AntValueB*10000;
		} //end while 2
		if ( ! interpolated ) {

					//if  ( ActValueA <= AntValueB) {
			        if (actA <= antB){
							//AntValueA=ActValueA;
						    std::cout<< "ActValueA <= AntValueB"<<std::endl;
						    ActValueA +=frequency;
						    std::cout<< "ActValueA="<< ActValueA<<std::endl;


					  // } else if ( ActValueA >= ActValueB ){
					   } else if ( actA >= actB ){
					  //	if ( ActValueA >= ActValueB ){
						  std::cout<< "ActValueA > ActValueB"<<std::endl;
						   newBMatrix.row(newContB) << B.row(contB);
						   std::cout<< "added B.row="<<B.row(contB)<<std::endl;
						   //std::cout<< "newBMatrix="<<newBMatrix<<std::endl;
						   newContB++;
						   std::cout<< "newContB="<<newContB<<std::endl;
		                   AntValueB=ActValueB;
		                   std::cout<< "AntValueB="<<AntValueB<<std::endl;
						   contB++;
						   std::cout<< "contB++="<<contB<<std::endl;
						   if (contB < B.rows()){
							   std::cout<< "B.row(contB)(0)="<<B.row(contB)(0)<<std::endl;
							   ActValueB=B.row(contB)(0);
						   }else{
								   endInterpolate = 1;
							   }


						   }


				}

	}
	B= newBMatrix.block(0,0,newContB,4);


}
void Interpolator::interpolate2SeriesB(int maxLine, MatrixXd& A, MatrixXd& B){

	// Supposed B less values than A
	double AntValueA=0,AntValueB=0,ActValueA=0,ActValueB=0;
	int contA=0,contB = 0, newContB=0;
	AntValueA=A.row(0)(0);
	AntValueB=B.row(0)(0);
	ActValueA=A.row(0)(0);
	ActValueB=B.row(1)(0);
	contA=1;
	contB=1;

	double newValue=0;
	MatrixXd newBMatrix (A.rows()+B.rows(),A.cols());
	newBMatrix.row(newContB)<< B.row(0);
	newContB++;
    int interpolated=0,endInterpolate = 0;
    long actB,actA,antB;
	//while ( (contB) <= B.rows() && (contA ) < A.rows()){// begin while 1
    while ( !endInterpolate && contA  < A.rows() && contB < B.rows()){// begin while 1
		//std::cout<<"Brows="<<B.rows()<<"ActValueA="<<ActValueA<<" ActValueB="<<ActValueB<<" AntValueB="<<AntValueB<< " contB="<<contB<<" contA="<<contA<<" newContB="<<newContB<<std::endl;
		interpolated = 0;
		actB = ActValueB*10000;
		actA = ActValueA*10000;
		antB = AntValueB*10000;
		//while (ActValueA < ActValueB && ActValueA > AntValueB  && (contA ) < A.rows()) { //begin while 2
		while (actA < actB && actA > antB  && (contA ) < A.rows()) { //begin while 2
			double y2= (B.row(contB-1))(1);
			double x = (A.row(contA-1))(0);
			double x2= (B.row(contB-1))(0);
			double y3= (B.row(contB))(1);
			double x3= (B.row(contB))(0);
			double xCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
			//std::cout<< "interpolo X"<<std::endl;
			//interpolate Y coordinate
			y2= (B.row(contB-1))(2);
			x = (A.row(contA-1))(0);
			x2= (B.row(contB-1))(0);
			y3= (B.row(contB))(2);
			x3= (B.row(contB))(0);
			double yCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
			//std::cout<< "interpolo Y"<<std::endl;
			//interpolate Z coordinate
			y2= (B.row(contB-1))(3);
			x = (A.row(contA-1))(0);
			x2= (B.row(contB-1))(0);
			y3= (B.row(contB))(3);
			x3= (B.row(contB))(0);
			double zCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
			//std::cout<< "interpolo X,y,Z"<<std::endl;
			Vector4d myNewVector(x,xCoordinate,yCoordinate,zCoordinate);
			//std::cout<< "myNewVector="<<myNewVector<<std::endl;
			//contAddedValues++;
            newBMatrix.row(newContB)<< myNewVector.transpose();
            newContB ++;

            ActValueA=A.row(contA)(0);
            actA = ActValueA*10000;
            contA++;
            interpolated = true;
		} //end while 2

		if ( ! interpolated && contA < A.rows()) {

			//if  ( ActValueA <= AntValueB) {
			if  ( actA <= antB) {
					//AntValueA=ActValueA;
				    //std::cout<< "ActValueA <= AntValueB"<<std::endl;
				    if (contA < A.rows()){
						ActValueA=A.row(contA)(0);
						contA++;
					}else{
						endInterpolate=1;
					}


			   }

			    /*

			      else if ( ActValueA == ActValueB  ){
					//if (contB < B.rows()){
					//	newBMatrix.row(newContB) << B.row(contB-1);
					//	newContB++;
					//	AntValueB=ActValueB;
					//	ActValueB=B.row(contB)(0);
					//	contB++;
					//}
				    newBMatrix.row(newContB) << B.row(contB);
				   	newContB++;
				   	AntValueB=ActValueB;
				   	contB++;
				    if (contB < B.rows()){
						ActValueB=B.row(contB)(0);
				   	} else {
				   		endInterpolate=1;
				   	}
				    //std::cout<< "ActValueA == ActValueB"<<std::endl;

                    //if (contA < A.rows()){
					//	ActValueA=A.row(contA)(0);
					//	contA++;
                    //}else{
                    //	endInterpolate=1;
                    //}

               */

			   //else if ( ActValueA >= ActValueB ){
			 else if ( actA >= actB ){
			  //	if ( ActValueA >= ActValueB ){
				   //std::cout<< "ActValueA > ActValueB"<<std::endl;
				   newBMatrix.row(newContB) << B.row(contB);
				   //std::cout<< "newBMatrix="<<newBMatrix<<std::endl;
				   newContB++;
				   //std::cout<< "newContB="<<newContB<<std::endl;
                   AntValueB=ActValueB;
                   //std::cout<< "AntValueB="<<AntValueB<<std::endl;
				   contB++;
				   //std::cout<< "contB++="<<contB<<std::endl;
				   if (contB < B.rows()){
					   //std::cout<< "B.row(contB)(0)="<<B.row(contB)(0)<<std::endl;
					   ActValueB=B.row(contB)(0);
				   }else{
						   endInterpolate = 1;
					   }


				   }


		}

		//std::cout<< "contB="<<contB<<std::endl;
		//std::cout<< "newContB="<<newContB<<std::endl;
		//std::cout<< "contA="<<contA<<std::endl;
		//std::cout<<"antes del final del while ActValueA="<<ActValueA<<" ActValueB="<<ActValueB<<" AntValueB="<<AntValueB<< " contB="<<contB<<" contA="<<contA<<" newContB="<<newContB<<std::endl;
		}//end While 1
	//std::cout<< "end while>>>>>>>>>>>>>>>"<<std::endl;

	//std::cout<< "newBMatrix="<<newBMatrix<<std::endl;
	//std::cout<< "contB="<<contB<<std::endl;
	//std::cout<< "B.rows()="<<B.rows()<<std::endl;
	while (contB < B.rows()){

			newBMatrix.row(newContB) << B.row(contB);
			newContB++;
			contB++;

		}

	//std::cout<< "newBMatrix="<<newBMatrix<<std::endl;
	//newBMatrix.row(newContB) << B.row(contB);
    //newContB++;
	B= newBMatrix.block(0,0,newContB,4);
	}

	/*
	int Bsize= B.rows();
	std::cout<< "Bsize="<<Bsize<<std::endl;
	int Asize= A.rows();
	std::cout<< "Asize="<<Asize<<std::endl;
	int i =1;
	int j , contAddedValues= 0;
	MatrixXd newBMatrix (A.rows()+B.rows(),A.cols());
    int contNewMatrix =0;
	while (i < Bsize ){
		std::cout<< "i="<<i<<std::endl;
		newBMatrix.row(contNewMatrix)<< B.row(i-1);
		contNewMatrix++;
		//std::cout<< "contNewMatrix="<<contNewMatrix<<std::endl;
		//std::cout<< "B.row(i-1)(0)="<<B.row(i-1)(0)<<std::endl;
		contAddedValues = 0;
		//j=i-1;
		while (j< Asize  && (A.row(j)(0) < B.row(i)(0)) ){

			//std::cout<< "i="<<i<< "j="<<j<<" (A.row(j)(0)="<<(A.row(j))(0)<<" B.row(i)(0)="<<(B.row(i))(0)<<" Asize="<<Asize<<"Bsize="<<Bsize<< "contNewMatrix="<<contNewMatrix<<std::endl;
			std::cout<< "antes del if"<<std::endl;
			if ((A.row(j)(0) > B.row(i-1)(0)) ) {
				std::cout<< "dentro del if"<<std::endl;
				//interpolate X coordinate
				double y2= (B.row(i-1))(1);
				double x = (A.row(j))(0);
				double x2= (B.row(i-1))(0);
				double y3= (B.row(i))(1);
				double x3= (B.row(i))(0);
				double xCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo X"<<std::endl;
				//interpolate Y coordinate
				y2= (B.row(i-1))(2);
				x = (A.row(j))(0);
				x2= (B.row(i-1))(0);
				y3= (B.row(i))(2);
				x3= (B.row(i))(0);
				double yCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo Y"<<std::endl;
				//interpolate Z coordinate
				y2= (B.row(i-1))(3);
				x = (A.row(j))(0);
				x2= (B.row(i-1))(0);
				y3= (B.row(i))(3);
				x3= (B.row(i))(0);
				double zCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo Z"<<std::endl;
				Vector4d myNewVector(x,xCoordinate,yCoordinate,zCoordinate);
				std::cout<< "myNewVector="<<myNewVector<<std::endl;
				//contAddedValues++;

				newBMatrix.row(contNewMatrix) << myNewVector.transpose();
				contNewMatrix++;
				std::cout<< "contNewMatrix="<<contNewMatrix<<std::endl;
			} else {

				std::cout<< "(A.row(j)(0) > B.row(i-1)(0) NO ES MENOR"<<std::endl;
			}
			std::cout<< "despues del if"<<std::endl;
			j++;
		}
		i++;

	}
	//std::cout<< "contNewMatrix="<<contNewMatrix<<std::endl;


	//std::cout<< "A="<<A<<std::endl;
	//std::cout<< "B="<<B<<std::endl;

	newBMatrix.row(contNewMatrix)<<B.row(i-1);
	contNewMatrix ++;
	//std::cout<< "newBMatrix="<<newBMatrix<<std::endl;

	B= newBMatrix.block(0,0,contNewMatrix,4);
	//std::cout<< "new B Matrix="<<B<<std::endl;


}
*/


void Interpolator::interpolate2Series(int maxLine, MatrixXd& A, MatrixXd& B){

	// Supposed B less values than A
	int Bsize= B.rows();
	std::cout<< "Bsize="<<Bsize<<std::endl;
	int Asize= A.rows();
	std::cout<< "Asize="<<Asize<<std::endl;
	int i =1;
	int j , contAddedValues= 0;
	MatrixXd newBMatrix (A.rows()+B.rows(),A.cols());
    int contNewMatrix =0;
	while (i < Bsize ){
		std::cout<< "i="<<i<<std::endl;
		newBMatrix.row(contNewMatrix)<< B.row(i-1);
		contNewMatrix++;
		//std::cout<< "contNewMatrix="<<contNewMatrix<<std::endl;
		//std::cout<< "B.row(i-1)(0)="<<B.row(i-1)(0)<<std::endl;
		contAddedValues = 0;
		//j=i-1;
		while (j< Asize  && (A.row(j)(0) < B.row(i)(0)) ){

			//std::cout<< "i="<<i<< "j="<<j<<" (A.row(j)(0)="<<(A.row(j))(0)<<" B.row(i)(0)="<<(B.row(i))(0)<<" Asize="<<Asize<<"Bsize="<<Bsize<< "contNewMatrix="<<contNewMatrix<<std::endl;
			std::cout<< "antes del if"<<std::endl;
			if ((A.row(j)(0) > B.row(i-1)(0)) ) {
				std::cout<< "dentro del if"<<std::endl;
				//interpolate X coordinate
				double y2= (B.row(i-1))(1);
				double x = (A.row(j))(0);
				double x2= (B.row(i-1))(0);
				double y3= (B.row(i))(1);
				double x3= (B.row(i))(0);
				double xCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo X"<<std::endl;
				//interpolate Y coordinate
				y2= (B.row(i-1))(2);
				x = (A.row(j))(0);
				x2= (B.row(i-1))(0);
				y3= (B.row(i))(2);
				x3= (B.row(i))(0);
				double yCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo Y"<<std::endl;
				//interpolate Z coordinate
				y2= (B.row(i-1))(3);
				x = (A.row(j))(0);
				x2= (B.row(i-1))(0);
				y3= (B.row(i))(3);
				x3= (B.row(i))(0);
				double zCoordinate = this->interpolateValue(x,x2,y2,x3,y3);
				std::cout<< "interpolo Z"<<std::endl;
				Vector4d myNewVector(x,xCoordinate,yCoordinate,zCoordinate);
				std::cout<< "myNewVector="<<myNewVector<<std::endl;
				//contAddedValues++;

				newBMatrix.row(contNewMatrix) << myNewVector.transpose();
				contNewMatrix++;
				std::cout<< "contNewMatrix="<<contNewMatrix<<std::endl;
			} else {

				std::cout<< "(A.row(j)(0) > B.row(i-1)(0) NO ES MENOR"<<std::endl;

			}
			std::cout<< "despues del if"<<std::endl;
			j++;
		}
		i++;

	}
	//std::cout<< "contNewMatrix="<<contNewMatrix<<std::endl;


	//std::cout<< "A="<<A<<std::endl;
	//std::cout<< "B="<<B<<std::endl;

	newBMatrix.row(contNewMatrix)<<B.row(i-1);
	contNewMatrix ++;
	//std::cout<< "newBMatrix="<<newBMatrix<<std::endl;

	B= newBMatrix.block(0,0,contNewMatrix,4);
	//std::cout<< "new B Matrix="<<B<<std::endl;


}

/*
void Interpolator::interpolate2Series(int maxLine, MatrixXd& A, MatrixXd& B){

 //testing inserting row
  std::cout <<"interpolate"<<std::endl;
  int sizeB = B.rows();
  double timeBPrevious = 0;
  MatrixXd myNewBMatrix (A.rows(),A.cols());
  int cont=0;
  std::cout <<"cont="<<cont<<std::endl;
  myNewBMatrix.row(0)<< B.row(0);
  std::cout <<"cont="<<cont<<std::endl;
  for (int i=1;i< sizeB ;i++){
	  //std::cout <<"i="<<i<<std::endl;
	  if ((B.row(i))(0) > (A.row(i))(0)){

		  //for x coordinate
		  double y2= (B.row(i-1))(1);
		  double x = (A.row(i))(0);
		  double x2= (B.row(i-1))(0);
		  double y3= (B.row(i))(1);
		  double x3= (B.row(i))(0);
		  double y = this->interpolateValue(x,x2,y2,x3,y3);
		  Vector4d myNewVector(x,y,y,y);
		  //this->insertRowInSequence(B,myNewVector,i);
          myNewBMatrix.row(i)<< myNewVector.transpose();
          myNewBMatrix.row(i+1)<< B.row(i);
          cont++;
          //std::cout <<"iSecret="<<iSecret<<std::endl;
		  //double newTimeB= (B.row(i-1).(0) + B.row(i).(0))/2;
		  //double newX = (B.row(i-1).(1) + B.row(i).(1))/2;
		  //double newY = (B.row(i-1).(2) + B.row(i).(2))/2;
		  //double newZ = (B.row(i-1).(3) + B.row(i).(3))/2;
		  //VectorX4d myNewVector(newTimeB,newX,newY,newZ);
		  //this.insertRowInSequence(B,myNewVector,i);


	  } else {
		  //myNewBMatrix.row(cont)<< B.row(i);
		  myNewBMatrix.row(i+cont)<< B.row(i);
	  }
	  //cont++;
  }
  std::cout <<"cont="<<cont<<std::endl;
  B = myNewBMatrix.block(0,0,myNewBMatrix.rows(),4);


}
*/

void Interpolator::modifyTime (double valueToAdd, MatrixXd& aMatrix){
	// aMatrix has 4 cols (time, x , y ,z) and N rows
	// we will add some value to the time column
	for (int i=0; i< aMatrix.rows(); i++){
		Vector4d myNewVector(((aMatrix.row(i))(0) + valueToAdd),(aMatrix.row(i))(1),(aMatrix.row(i))(2),(aMatrix.row(i))(3));
		//contAddedValues++;



		aMatrix.row(i)<<myNewVector.transpose();

	}
}


void Interpolator::reduceSequence(int step, MatrixXd& aMatrix){
	/* This method deletes a number on lines of a matrix
	 * The lines are selected by step
	 * if step is 2, lines 2,4,6 .. 10,12 ..( n+2) will be deleted. Equivalent to delete 50%
	 * if step is 3, lines 3,6,9, 12 ... (n+3) will be deleted. Equivalent to delete 30%
	 *
	 *
	 *  */
	if (step < 2) {
		std::cout <<"Error:ReduceSequence: parameter step must be greater than 1, step ="<<step<<std::endl;


	} else {
		const double MaxValue=99999999999;
		for (int i=step;i < aMatrix.rows()-step;i +=step) {
			aMatrix(i,0)=MaxValue;//set the time to MaxValue
			std::cout <<"aMatrix(iSecret,0)="<<aMatrix(i,0)<<"i="<<i<<std::endl;
		}
		MatrixXd myNewMatrix (aMatrix.rows(),4);
		int cont=0;
		for (int i=0;i<aMatrix.rows();i++){
			std::cout <<"aMatrix(i,0)="<<aMatrix(i,0)<<std::endl;
			if (aMatrix(i,0)< MaxValue){
				std::cout <<"encontrado="<<aMatrix(i,0)<<std::endl;
				myNewMatrix.row(cont) <<  aMatrix.row(i);
				cont++;
			}

		}


		std::cout <<"cont="<<cont<<std::endl;
		aMatrix = myNewMatrix.block(0,0,cont,4);
		std::cout<<"After reducing, number of lines of matrix ="<<aMatrix.rows();
		//std::cout <<"aMatrix="<<aMatrix<<std::endl;
        }

}


void Interpolator::reduceSequence(int maxLine, MatrixXd& aMatrix , int numberToDelete){
	/*This method deletes a number of lines of a matrix
	 * The lines are selected randomly
	 * The shrunk matrix,  is returned.
	 *
	 * Select randomly the rows to delete and set to MaxValue
	 * At the end create a new matrix without the rows that contain MaxValue
	 */

	int iSecret;
    const double MaxValue=99999999999;
	/* initialize random seed: */
	srand (time(NULL));
    for (int i=0;i<numberToDelete;i++) {
		/* generate secret number between 1 and 10: */
		iSecret = rand() % aMatrix.rows() ;
		std::cout <<"iSecret="<<iSecret<<std::endl;
		while (aMatrix(iSecret,0)== MaxValue) {//skip the rows that are already set to max value
			iSecret = (iSecret + 1) % aMatrix.rows();
		}
		aMatrix(iSecret,0)=MaxValue;//set the time to MaxValue
		std::cout <<"aMatrix(iSecret,0)="<<aMatrix(iSecret,0)<<std::endl;
    }
    MatrixXd myNewMatrix (aMatrix.rows(),4);
    int cont=0;
    for (int i=0;i<aMatrix.rows();i++){
    	std::cout <<"aMatrix(i,0)="<<aMatrix(i,0)<<std::endl;
    	if (aMatrix(i,0)< MaxValue){
    		std::cout <<"encontrado="<<aMatrix(i,0)<<std::endl;
    		myNewMatrix.row(cont) <<  aMatrix.row(i);
    		cont++;
    	}

    }


    std::cout <<"cont="<<cont<<std::endl;
    aMatrix = myNewMatrix.block(0,0,cont,4);
    //std::cout <<"aMatrix="<<aMatrix<<std::endl;



}


int main(){

	std::cout <<"	Starting main of  Interpolator="<<std::endl;
	Interpolator myInterpolator;
	AjusteTiempo micorrelador ;

	double offset = 28;
	int maxLine = 2500;
	int intervalo = 200;
	MatrixXd readingA (maxLine,4);
	MatrixXd readingB (maxLine,4);
    //Read Files for sequence A and Sequence B

	std::cout <<"1="<<std::endl;
	std::cout << std::setprecision(4) << std::fixed;
	std::ifstream infileA( "/home/tfm3/workspace/Interpolator/miEntradaA.txt" );
	//inputFile >> std::setprecision(6) >> std::fixed;
	std::cout << std::setprecision(4) << std::fixed;
	std::ifstream infileB( "/home/tfm3/workspace/Interpolator/miEntradaB.txt" );
	//inputFile >> std::setprecision(6) >> std::fixed;
	std::cout <<"2="<<std::endl;
	std::cout << std::setprecision(4) << std::fixed;
	std::ofstream outA( "/home/tfm3/workspace/Interpolator/miSalidaA.txt" );
	outA << std::setprecision(4) << std::fixed;
	std::cout << std::setprecision(4) << std::fixed;

	std::ofstream outB( "/home/tfm3/workspace/Interpolator/miSalidaB.txt" );
	outB << std::setprecision(4) << std::fixed;

	std::ofstream outBbeforeInterpolate( "/home/tfm3/workspace/Interpolator/myOutputBbeforeInterpolate.txt" );
	outBbeforeInterpolate << std::setprecision(4) << std::fixed;

	std::ofstream outAbeforeInterpolate( "/home/tfm3/workspace/Interpolator/myOutputAbeforeInterpolate.txt" );
	outAbeforeInterpolate << std::setprecision(4) << std::fixed;

	std::cout << std::setprecision(4) << std::fixed;
	std::ofstream outRegresion( "/home/tfm3/workspace/Interpolator/miSalidaRegresion.txt" );
	outRegresion << std::setprecision(4) << std::fixed;
	std::cout <<"3="<<std::endl;

	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	int contLin=0;
	std::cout <<"4="<<std::endl;
	while ( (infileA >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
		        std::cout <<"contLin="<<contLin<<std::endl;
			    readingA.row(contLin)<<timestamp, rx,ry,rz;
			    //std::cout <<"contLin="<<contLin<<std::endl;
				contLin ++;


	}
    infileA.close();
	MatrixXd A (contLin,4);
	A = readingA.block(0,0,contLin,4);

	contLin=0;

	while ( (infileB >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
				    readingB.row(contLin)<<timestamp, rx,ry,rz;

					contLin ++;

	}
	infileB.close();
	MatrixXd B (contLin,4);
	B = readingB.block(0,0,contLin,4);



	/*MatrixXd m(4,4);
    m <<  1, 2, 3, 4,
	      5, 6, 7, 8,
	      9,10,11, 8,
          12,13,14, 15;
    Vector4d v(4,4,4,4);
    */

    //MatrixXd newMatrix (5,3);

    //std::cout <<"newMatrix="<<newMatrix<<std::endl;

	//myInterpolator.interpolate(maxLine, A,B);
	//std::cout <<"B="<<B<<std::endl;




    //myInterpolator.insertRowInSequence(m,v,2);
    //std::cout <<"m="<<m<<std::endl;

	//REDUCING , SHRINKING DATA
    //int valuesToReduce =1250;
	//myInterpolator.reduceSequence(maxLine,B, valuesToReduce);

	myInterpolator.reduceSequence(2,B);

	//int valuesToReduce2 = 1000;
	//myInterpolator.reduceSequence(maxLine,A, valuesToReduce2);

	//myInterpolator.interpolateSerieToFrequency(0.005,A);

    std::cout <<"before interpolate"<<std::endl;

    // Save Serie B into a file , before interpolate.  Later compare graphically
    for (int i= 0; i< B.rows(); i++){
    					//std::cout <<"i="<<i<<std::endl;
    					//std::cout <<"centered.row(i)="<<B.row(i)<<std::endl;


    					VectorXd bRow = B.row(i);
    					outBbeforeInterpolate << bRow(0)<<" "<<bRow(1)<<" "<<bRow(2)<<" "<<bRow(3)<<"\n";

    }
    outBbeforeInterpolate.close();

    for (int i= 0; i< A.rows(); i++){
        					//std::cout <<"i="<<i<<std::endl;
        					//std::cout <<"centered.row(i)="<<B.row(i)<<std::endl;


        					VectorXd aRow = A.row(i);
        					outAbeforeInterpolate << aRow(0)<<" "<<aRow(1)<<" "<<aRow(2)<<" "<<aRow(3)<<"\n";

        }
        outAbeforeInterpolate.close();


    double step = 0.1;
    double limit = 20;
    double delayMax=0, rMax=0;
    MatrixXd finalBMatrix (B.rows()+A.rows(),B.cols());
    double i = 0;

    /*
    MatrixXd newBMatrix (B.rows()+A.rows(),B.cols());
    MatrixXd newAMatrix (A.rows()+B.rows(),A.cols());
    newBMatrix= B.block(0,0,B.rows(),4);
    newAMatrix= A.block(0,0,A.rows(),4);
    myInterpolator.interpolate2SeriesB(maxLine, A,newBMatrix);
    myInterpolator.interpolate2SeriesB(maxLine,B,newAMatrix);
    A=newAMatrix.block(0,0,newAMatrix.rows(),4);
    B=newBMatrix.block(0,0,newBMatrix.rows(),4);
    */


    //calculating regresion
    for (double i =-limit;i<limit;i+=step){

        //Start calculating crossCorrelation
    	//First: make a copy of B matrix.
    	//In this case we move Data B , on each iteration , from -limit to limit, trying to find the best correlation value
    	MatrixXd newBMatrix (B.rows()+A.rows(),B.cols());
    	MatrixXd newInterpolated (B.rows()+A.rows(),B.cols());
    	newBMatrix= B.block(0,0,B.rows(),4);
		//myInterpolator.modifyTime(i,B);
    	myInterpolator.modifyTime(i,newBMatrix);
		std::cout <<"Antes de interpolar:newBMatrix.rows()="<<newBMatrix.rows()<<std::endl;
		//myInterpolator.interpolate2Series(maxLine, A,B);
		myInterpolator.interpolate2SeriesB(maxLine, A,newBMatrix);
        //myInterpolator.interpolate2Series(maxLine, A,newBMatrix);
		//newInterpolated=myInterpolator.interpolateAoverB(A,newBMatrix);
		//newBMatrix= newInterpolated.block(0,0,newInterpolated.rows(),4);
		std::cout <<"Despues de interpolar:newBMatrix.rows()="<<newBMatrix.rows()<<std::endl;
		std::cout <<"after interpolate"<<std::endl;
	    std::cout <<"newBMatrix has"<<newBMatrix.rows()<<"rows"<<std::endl;
		//double r=micorrelador.calcularAutocorrelacion4(A.rows(), 100,step, A,  newBMatrix);
	    double r=micorrelador.calcularAutocorrelacion4(A.rows(), 100,0, A,  newBMatrix);
		std::cout <<"regresion="<<r<<std::endl;
		outRegresion<< r << " " << i << std::endl;
		std::cout <<"newBMatrix after regresion"<<newBMatrix.rows()<<"rows"<<std::endl;
		std::cout <<"fabs(r)"<<fabs(r)<<std::endl;
		std::cout <<"rMax"<<rMax<<std::endl;

		if ((fabs(r) <= 1 )&& (fabs(r) >= rMax)){

		        	  rMax=r;
		        	  delayMax=i;
		        	  finalBMatrix=newBMatrix.block(0,0,newBMatrix.rows(),4);
		        	  std::cout <<"finalBMatrixafter regresion++++++++++++++++++++++++++++++"<<finalBMatrix.rows()<<"rows"<<std::endl;

	    }


		std::cout <<"i----------------------------------------------------"<<i<<std::endl;
    }

    B=finalBMatrix.block(0,0,finalBMatrix.rows(),4);
    std::cout <<"rMax="<<rMax<<std::endl;
    std::cout <<"delayMax="<<delayMax<<std::endl;
    std::cout <<"b.rows()="<<B.rows()<<std::endl;
    std::cout <<"a.rows()="<<A.rows()<<std::endl;
    outRegresion.close();

    // end calculating REGRESSION

    //Save data A
	for (int i= 0; i< A.rows(); i++){
				//std::cout <<"i="<<i<<std::endl;
				//std::cout <<"centered.row(i)="<<A.row(i)<<std::endl;

				VectorXd aRow = A.row(i);
				outA << aRow(0)<<" "<<aRow(1)<<" "<<aRow(2)<<" "<<aRow(3)<<"\n";



		}
	// Save data B
	for (int i= 0; i< B.rows(); i++){
					//std::cout <<"i="<<i<<std::endl;
					//std::cout <<"centered.row(i)="<<B.row(i)<<std::endl;


					VectorXd bRow = B.row(i);
					outB << bRow(0)<<" "<<bRow(1)<<" "<<bRow(2)<<" "<<bRow(3)<<"\n";

			}
	outA.close();
	outB.close();


	std::cout <<"FIN PROGRAMA>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<"b.rows()="<<B.rows()<<"a.rows()="<<A.rows()<<std::endl;
}


