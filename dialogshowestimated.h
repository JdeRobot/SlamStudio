#ifndef DIALOGSHOWESTIMATED_H
#define DIALOGSHOWESTIMATED_H
#include <QDialog>
#include "datadialogshowestimated.h"
#include <QtWidgets>
#include <iostream>
//#include "mainwindow.h"

class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;
class QTextBox;
class MainWindow;

class DialogShowEstimated : public QDialog
{
    Q_OBJECT

public:
    DialogShowEstimated(QWidget *parent = 0);
    //DialogScalaTraslaRota(QWidget *parent = 0, DataDialogScalaTraslaRota* aModel );
    MainWindow *parent;

private:
    QLabel *labelTitle;
    QLabel *label2;
    QLabel *labelGaussNoise;
    QLabel *labelCosmicNoise;
    QLineEdit *saveFile;
    QLabel *labScaleX;
    QLineEdit *scaleX;
    QLabel *labScaleY;
    QLineEdit *scaleY;
    QLabel *labScaleZ;
    QLineEdit *scaleZ;
    QLabel *labTraslaX;
    QLineEdit *traslaX;
    QLabel *labTraslaY;
    QLineEdit *traslaY;
    QLabel *labTraslaZ;
    QLineEdit *traslaZ;
    QLabel *labTimeOffset;
    QLineEdit *timeOffset;

    QLabel *labRMax;
    QLineEdit *rMax; // to store max correlation value, the one that indicates which is the value of the offset

    QLabel *labYaw;
    QLineEdit *yaw;
    QLabel *labPitch;
    QLineEdit *pitch;
    QLabel *labRoll;
    QLineEdit *roll;

    QLabel *labRMSE;
    QLineEdit *RMSE;

    QLabel *labRotationMatrix;
    QTextEdit *rotationMatrix;
    QLabel *labPcaA;
    QTextEdit *pcaA;
    QLabel *labPcaB;
    QTextEdit *pcaB;
    QLabel *labFrequency;
    QTextEdit *frequency;



    QPushButton *buttonOK;
    QPushButton *buttonCancel;
    DataDialogShowEstimated* dialogModel;
public slots:
    void onOK();
    void onCancel();


 public:
    void setDataDialog(DataDialogShowEstimated* aModel);
    DataDialogShowEstimated* getDataDialog();

};

#endif // DIALOGDATAESTIMATED_H
