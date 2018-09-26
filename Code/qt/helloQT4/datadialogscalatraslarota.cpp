#include "datadialogscalatraslarota.h"
DataDialogScalaTraslaRota::DataDialogScalaTraslaRota(){

}
DataDialogScalaTraslaRota::DataDialogScalaTraslaRota(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX, double rY, double rZ,double gN , double cN){
    scaleX=sX;
    scaleY=sY;
    scaleZ=sZ;
    traslaX=tX;
    traslaY=tY;
    traslaZ=tZ;
    rotaX=rX;
    rotaY=rY;
    rotaZ=rZ;
    gNoise=gN;
    cNoise=cN;

}

void DataDialogScalaTraslaRota::updateData(double sX,double sY, double sZ,double tX,double tY, double tZ, double rX, double rY, double rZ,double gN , double cN){
    scaleX=sX;
    scaleY=sY;
    scaleZ=sZ;
    traslaX=tX;
    traslaY=tY;
    traslaZ=tZ;
    rotaX=rX;
    rotaY=rY;
    rotaZ=rZ;
    gNoise=gN;
    cNoise=cN;
}
double DataDialogScalaTraslaRota::getScaleX(){
    return scaleX;
};
double DataDialogScalaTraslaRota::getScaleY(){
    return scaleY;
};
double DataDialogScalaTraslaRota::getScaleZ(){
    return scaleZ;
};
double DataDialogScalaTraslaRota::getTraslaX(){
    return traslaX;
};
double DataDialogScalaTraslaRota::getTraslaY(){
    return traslaY;
};
double DataDialogScalaTraslaRota::getTraslaZ(){
    return traslaZ;
};
double DataDialogScalaTraslaRota::getRotaX(){
    return rotaX;
};
double DataDialogScalaTraslaRota::getRotaY(){
    return rotaY;
};
double DataDialogScalaTraslaRota::getRotaZ(){
    return rotaZ;
};
double DataDialogScalaTraslaRota::getGaussianNoiseDeviation(){
    return gNoise;
};
double DataDialogScalaTraslaRota::getCosmicNoiseDeviation(){
    return cNoise;
};

void DataDialogScalaTraslaRota::setScaleX(double aValue){
    scaleX=aValue;
};
void DataDialogScalaTraslaRota::setScaleY(double aValue){
    scaleY=aValue;
};
void DataDialogScalaTraslaRota::setScaleZ(double aValue){
    scaleZ=aValue;
};
void DataDialogScalaTraslaRota::setTraslaX(double aValue){
    traslaX=aValue;
};
void DataDialogScalaTraslaRota::setTraslaY(double aValue){
    traslaY=aValue;
};
void DataDialogScalaTraslaRota::setTraslaZ(double aValue){
    traslaZ=aValue;
};
void DataDialogScalaTraslaRota::setRotaX(double aValue){
    rotaX=aValue;
};
void DataDialogScalaTraslaRota::setRotaY(double aValue){
    rotaY=aValue;
};
void DataDialogScalaTraslaRota::setRotaZ(double aValue){
    rotaZ=aValue;
};
void DataDialogScalaTraslaRota::setGaussianNoiseDeviation(double aValue){
    gNoise=aValue;
};
void DataDialogScalaTraslaRota::setCosmicNoiseDeviation(double aValue){
    cNoise=aValue;
};
