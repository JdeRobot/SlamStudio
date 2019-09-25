#include "datadialogparameters.h"


DataDialogParameters::DataDialogParameters()
= default;
DataDialogParameters::DataDialogParameters(int maxLines, double step,double windowSize){
    MAXLINES=maxLines;
    stepOffset=step;
    offsetWindow=windowSize;
}


int DataDialogParameters::getMAXLINES(){
    return MAXLINES;
}
double DataDialogParameters::getOffsetWindow(){
    return offsetWindow;
}
double DataDialogParameters::getStepOffset(){
    return stepOffset;
}

void DataDialogParameters::setStepOffset(double step){
    stepOffset=step;
}
void DataDialogParameters::setMAXLINES(int maxLines){
    MAXLINES=maxLines;
}
void DataDialogParameters::setOffsetWindow(double windowSize){
    offsetWindow=windowSize;

}
void DataDialogParameters::updateData(int numLines, double stepOffset, double winOffset){
    this->setMAXLINES(numLines);
    this->setOffsetWindow(winOffset);
    this->setStepOffset(stepOffset);
}
