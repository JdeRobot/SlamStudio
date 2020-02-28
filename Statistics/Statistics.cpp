#include <iostream>
#include <ctime>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include "Eigen/Dense"
#include "Eigen/SVD"
using namespace Eigen;
#include <limits>
#include <stdbool.h>
//#include "../RegistradorHorn/RegistradorHorn.h"
#include "Statistics.h"
Statistics::Statistics(){
   errorRows << 1,1,1;
}
Statistics::Statistics(MatrixXd data1,MatrixXd data2){
   MatrixXd error1 = data1 - data2;
   errorRows = sqrt(error1.array().pow(2));
}
MatrixXd Statistics::calculateDiffError (MatrixXd data1, MatrixXd data2){
	// Find the error
		    	MatrixXd err = data1 - data2;
		    	MatrixXd err2 = sqrt(err.array().pow(2));
		    	return err2;


}

double Statistics:: RMSE (MatrixXd data1){

	MatrixXd err2 = data1.array().pow(2);
	return sqrt(err2.sum()/err2.rows());
}

double Statistics:: mean (MatrixXd data1){
    return data1.mean();
}
double Statistics:: median (MatrixXd data1){
    return data1.mean();
}
double Statistics:: max (MatrixXd data1){
    return data1.maxCoeff();
}
double Statistics:: min(MatrixXd data1){
    return data1.minCoeff();
}
MatrixXd Statistics::getErrorRows(){
    return errorRows;
}
