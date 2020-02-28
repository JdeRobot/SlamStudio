#include "datadialogshowestimated.h"
DataDialogShowEstimated::DataDialogShowEstimated()
= default;
DataDialogShowEstimated::DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3,double tOffset,double rm,int diType,Eigen::MatrixXd mpcaA,Eigen::MatrixXd mpcaB){
    scaleX=sX;
    scaleY=sY;
    scaleZ=sZ;
    traslaX=tX;
    traslaY=tY;
    traslaZ=tZ;
  timeOffset=tOffset;
    rmax=rm;
    dialogType=diType;

}

DataDialogShowEstimated::DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3,double tOffset,double rm,int diType,double rmse){
    scaleX=sX;
    scaleY=sY;
    scaleZ=sZ;
    traslaX=tX;
    traslaY=tY;
    traslaZ=tZ;
  timeOffset=tOffset;
    rmax=rm;
    RMSE=rmse;
    dialogType=diType;





}

DataDialogShowEstimated::DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double y, double p, double r,double tOffset,double rm,int diType,double rmse){
    scaleX=sX;
    scaleY=sY;
    scaleZ=sZ;
    traslaX=tX;
    traslaY=tY;
    traslaZ=tZ;
    yaw=y;
    pitch=p;
    roll=r;
    timeOffset=tOffset;
    rmax=rm;
    RMSE=rmse;
    dialogType=diType;





}

double DataDialogShowEstimated::getScaleX(){
    return scaleX;
};
double DataDialogShowEstimated::getScaleY(){
    return scaleY;
};
double DataDialogShowEstimated::getScaleZ(){
    return scaleZ;
};
double DataDialogShowEstimated::getTraslaX(){
    return traslaX;
};
double DataDialogShowEstimated::getTraslaY(){
    return traslaY;
};
double DataDialogShowEstimated::getTraslaZ(){
    return traslaZ;
};
double DataDialogShowEstimated::getTimeOffset(){
    return timeOffset;
};
double DataDialogShowEstimated::getRMax(){
    return rmax;
};
double DataDialogShowEstimated::getRMSE(){
    return RMSE;
};
double DataDialogShowEstimated::getYaw(){
    return yaw;
};
double DataDialogShowEstimated::getPitch(){
    return pitch;
};
double DataDialogShowEstimated::getRoll(){
    return roll;
};

int DataDialogShowEstimated::getDialogType(){
    return dialogType;
};

