/*
 * Statistics.h
 *
 *  Created on: Dec 11, 2018
 *      Author: tfm3
 */

#ifndef MYSTATISTICS_H
#define MYSTATISTICS_H
#include <iostream>
#include "Eigen/Dense"
//#include "../AjusteTiempo/AjusteTiempo.h"
using namespace Eigen;
//using namespace std;

class Statistics {
private:
    MatrixXd errorRows;
public:
   Statistics ();
   Statistics (MatrixXd data1, MatrixXd data2);
   MatrixXd calculateDiffError(MatrixXd data1, MatrixXd data2);
   double RMSE(MatrixXd data1);
   double mean(MatrixXd data1);
   double median(MatrixXd data1);
   double max(MatrixXd data1);
   double min(MatrixXd data1);
   MatrixXd getErrorRows();
   //void estimatePCAOrder(MatrixXd& A, MatrixXd& B);





};




#endif /* MYSTATISTICS_H */
