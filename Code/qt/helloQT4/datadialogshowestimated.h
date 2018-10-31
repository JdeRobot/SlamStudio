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

double rotX1;
double rotX2;
double rotX3;

double rotY1;
double rotY2;
double rotY3;

double rotZ1;
double rotZ2;
double rotZ3;
double timeOffset;
double rmax;
int dialogType; //if 1 Dialog will show estimated transformation from DatasetA to new DataSetB
                //if 2 Dialog will show estimated transformation from new DatasetB to  DataSetA

Eigen::MatrixXd mPcaA;
Eigen::MatrixXd mPcaB;

public:
DataDialogShowEstimated();
DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3, double tOffset,double rm,int labelTitle,Eigen::MatrixXd mPcaA,Eigen::MatrixXd mPcaB);
void updateData(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3, double tOffset,double rm,int labelTitle,Eigen::MatrixXd mPcaA,Eigen::MatrixXd mPcaB);
//DataDialogScalaTraslaRota(char* sX, char* sY, char* sZ,char* tX,char* tY, char* tZ, char* rX, char* rY, char* rZ);
double getScaleX();
double getScaleY();
double getScaleZ();
double getTraslaX();
double getTraslaY();
double getTraslaZ();
double getRotaX1();
double getRotaX2();
double getRotaX3();
double getRotaY1();
double getRotaY2();
double getRotaY3();
double getRotaZ1();
double getRotaZ2();
double getRotaZ3();
double getRMax();
double getTimeOffset();
int getDialogType();

Eigen::MatrixXd getPcaA();
Eigen::MatrixXd getPcaB();

void setScaleX(double aValue);
void setScaleY(double aValue);
void setScaleZ(double aValue);
void setTraslaX(double aValue);
void setTraslaY(double aValue);
void setTraslaZ(double aValue);
void setRotaX1(double aValue);
void setRotaX2(double aValue);
void setRotaX3(double aValue);
void setRotaY1(double aValue);
void setRotaY2(double aValue);
void setRotaY3(double aValue);
void setRotaZ1(double aValue);
void setRotaZ2(double aValue);
void setRotaZ3(double aValue);
void setTimeOffset(double aValue);
void setRMax(double aValue);
void setDialogType(int aValue);

void setPcaA(Eigen::MatrixXd aMatrix);
void setPcaB(Eigen::MatrixXd aMatrix);






};
#endif // DATADIALOGSHOWESTIMATED_H
