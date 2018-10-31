#include "datadialogshowestimated.h"
DataDialogShowEstimated::DataDialogShowEstimated(){

}
DataDialogShowEstimated::DataDialogShowEstimated(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3,double tOffset,double rm,int diType,Eigen::MatrixXd mpcaA,Eigen::MatrixXd mpcaB){
    scaleX=sX;
    scaleY=sY;
    scaleZ=sZ;
    traslaX=tX;
    traslaY=tY;
    traslaZ=tZ;
    rotX1=rX1;
    rotY1=rY1;
    rotZ1=rZ1;
    rotX2=rX2;
    rotY2=rY2;
    rotZ2=rZ2;
    rotX3=rX3;
    rotY3=rY3;
    rotZ3=rZ3;
    timeOffset=tOffset;
    rmax=rm;
    dialogType=diType;

    mPcaA=mpcaA;
    mPcaB=mpcaB;



}

void DataDialogShowEstimated::updateData(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX1, double rY1, double rZ1,double rX2, double rY2, double rZ2,double rX3, double rY3, double rZ3,double tOffset,double rm,int diType,Eigen::MatrixXd mpcaA,Eigen::MatrixXd mpcaB){
    scaleX=sX;
    scaleY=sY;
    scaleZ=sZ;
    traslaX=tX;
    traslaY=tY;
    traslaZ=tZ;
    rotX1=rX1;
    rotY1=rY1;
    rotZ1=rZ1;
    rotX2=rX2;
    rotY2=rY2;
    rotZ2=rZ2;
    rotX3=rX3;
    rotY3=rY3;
    rotZ3=rZ3;
    timeOffset=tOffset;
    rmax = rm;
    dialogType=diType;
    mPcaA=mpcaA;
    mPcaB=mpcaB;

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
double DataDialogShowEstimated::getRotaX1(){
    return rotX1;
};
double DataDialogShowEstimated::getRotaY1(){
    return rotY1;
};
double DataDialogShowEstimated::getRotaZ1(){
    return rotZ1;
};
double DataDialogShowEstimated::getRotaX2(){
    return rotX2;
};
double DataDialogShowEstimated::getRotaY2(){
    return rotY2;
};
double DataDialogShowEstimated::getRotaZ2(){
    return rotZ2;
};
double DataDialogShowEstimated::getRotaX3(){
    return rotX3;
};
double DataDialogShowEstimated::getRotaY3(){
    return rotY3;
};
double DataDialogShowEstimated::getRotaZ3(){
    return rotZ3;
};
double DataDialogShowEstimated::getTimeOffset(){
    return timeOffset;
};
double DataDialogShowEstimated::getRMax(){
    return rmax;
};

int DataDialogShowEstimated::getDialogType(){
    return dialogType;
};


Eigen::MatrixXd DataDialogShowEstimated::getPcaA(){
    return mPcaA;
}
Eigen::MatrixXd DataDialogShowEstimated::getPcaB(){
    return mPcaB;
}





void DataDialogShowEstimated::setScaleX(double aValue){
    scaleX=aValue;
};
void DataDialogShowEstimated::setScaleY(double aValue){
    scaleY=aValue;
};
void DataDialogShowEstimated::setScaleZ(double aValue){
    scaleZ=aValue;
};
void DataDialogShowEstimated::setTraslaX(double aValue){
    traslaX=aValue;
};
void DataDialogShowEstimated::setTraslaY(double aValue){
    traslaY=aValue;
};
void DataDialogShowEstimated::setTraslaZ(double aValue){
    traslaZ=aValue;
};
void DataDialogShowEstimated::setRotaX1(double aValue){
    rotX1=aValue;
};
void DataDialogShowEstimated::setRotaY1(double aValue){
    rotY1=aValue;
};
void DataDialogShowEstimated::setRotaZ1(double aValue){
    rotZ1=aValue;
};
void DataDialogShowEstimated::setRotaX2(double aValue){
    rotX2=aValue;
};
void DataDialogShowEstimated::setRotaY2(double aValue){
    rotY2=aValue;
};
void DataDialogShowEstimated::setRotaZ2(double aValue){
    rotZ2=aValue;
};
void DataDialogShowEstimated::setRotaX3(double aValue){
    rotX3=aValue;
};
void DataDialogShowEstimated::setRotaY3(double aValue){
    rotY3=aValue;
};
void DataDialogShowEstimated::setRotaZ3(double aValue){
    rotZ3=aValue;
};
void DataDialogShowEstimated::setRMax(double aValue){
    rmax=aValue;
};

void DataDialogShowEstimated::setDialogType(int aValue){
    dialogType=aValue;
}
void DataDialogShowEstimated::setPcaA(Eigen::MatrixXd aMatrix){
    mPcaA=aMatrix;
}

void DataDialogShowEstimated::setPcaB(Eigen::MatrixXd aMatrix){
    mPcaB=aMatrix;
}
