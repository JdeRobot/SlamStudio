#include "configuration.h"

int Configuration::maxLines=50000;;
double Configuration::windowOffset=3;
double Configuration::stepOffset=1/100.0;

Configuration::Configuration()
{
  maxLines=50000;
  windowOffset=3;
  stepOffset=1/100.0;
}


void Configuration::setMaxLines(int numRows){
    maxLines=numRows;
};
void Configuration::setWindowOffset(int winOffset){
    windowOffset=winOffset;
};
void Configuration::setStepOffset(double step){
    stepOffset=step;
};
int Configuration::getMaxLines(){
     return maxLines;
};
double Configuration::getWindowOffset(){
     return windowOffset;
};
double Configuration::getStepOffset(){
     return stepOffset;
};
