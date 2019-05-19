#ifndef CONFIGURATION_H
#define CONFIGURATION_H


class Configuration
{
private:
    static int maxLines;
    static double windowOffset;
    static double stepOffset;
public:

    Configuration();
    static void setMaxLines(int);
    static void setWindowOffset(int winOffset);
    static void setStepOffset(double stepOffset);
    static int getMaxLines();
    static double getWindowOffset();
    static double getStepOffset();


};

#endif // CONFIGURATION_H
