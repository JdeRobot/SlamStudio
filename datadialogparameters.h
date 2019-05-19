#ifndef DATADIALOGPARAMETERS_H
#define DATADIALOGPARAMETERS_H
#include <string>

class DataDialogParameters
{
private:
    int MAXLINES; //LIMIT FOR THE MAXIMUM NUMBER OF LINES, EITHER INCLUED ON A FILE OR AS RESULT OF INTERPOLATION
    double offsetWindow; //size of the window to calculate offset . Example from -3 to +3
    double stepOffset;    // steps to increase the calculus of the offset . Example 0.01
public:

    DataDialogParameters();
    DataDialogParameters(int maxLines,double stepOffset,double offsetWindow);
    int getMAXLINES();
    double getOffsetWindow();
    double getStepOffset();

    void setStepOffset(double);
    void setMAXLINES(int);
    void setOffsetWindow(double);
    void updateData(int numLines, double winOffset, double stepOffset);


};

#endif // DATADIALOGPARAMETERS_H
