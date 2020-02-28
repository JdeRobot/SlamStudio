#ifndef DATADIALOGSHOWESTIMATED_H
#define DATADIALOGSHOWESTIMATED_H
#include <string>
#include "Eigen/Dense"
class QString;
class DataDialogShowEstimated{
private:

double scaleX;
double scaleY;
double scaleZ;

double traslaX;

double traslaY;

double traslaZ;

  double timeOffset;
double rmax;
double yaw;
double pitch;
double roll;
double RMSE; //Root Mean Square Error
int dialogType; //if 1 Dialog will show estimated transformation from DatasetA to new DataSetB
                //if 2 Dialog will show estimated transformation from new DatasetB to  DataSetA

 public:
DataDialogShowEstimated();
DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3, double tOffset,double rm,int labelTitle,Eigen::MatrixXd mPcaA,Eigen::MatrixXd mPcaB);
DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3, double tOffset,double rm,int labelTitle,double rmse);
DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double Yaw, double Pitch,double Roll, double tOffset,double rm,int labelTitle,double rmse);

  //DataDialogScalaTraslaRota(char* sX, char* sY, char* sZ,char* tX,char* tY, char* tZ, char* rX, char* rY, char* rZ);
double getScaleX();
double getScaleY();
double getScaleZ();
double getTraslaX();
double getTraslaY();
double getTraslaZ();
  double getRMax();
double getTimeOffset();
int getDialogType();
double getRMSE();
double getYaw();
double getPitch();
double getRoll();

};
#endif // DATADIALOGSHOWESTIMATED_H
