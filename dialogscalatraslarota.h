#ifndef DIALOGSCALATRASLAROTA_H
#define DIALOGSCALATRASLAROTA_H
#include <QDialog>
#include <QtWidgets>
#include <iostream>

#include "datadialogscalatraslarota.h"

//#include "mainwindow.h"

class QCheckBox;
class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextBox;
class MainWindow;
class QRadioButton;
class QCheckBox;

class DialogScalaTraslaRota : public QDialog
{
    Q_OBJECT

public:
    DialogScalaTraslaRota(QWidget *parent = 0);
    MainWindow *parent;

private:
    QLabel *label;
    QLabel *label2;
    QLabel *labelGaussNoise;
    QLabel *labelCosmicNoise;
    QLineEdit *saveFile{};
    QLabel *labScaleX;
    QLineEdit *scaleX;
    QLabel *labScaleY{};
    QLineEdit *scaleY;
    QLabel *labScaleZ{};
    QLineEdit *scaleZ;
    QLabel *labTraslaX;
    QLineEdit *traslaX;
    QLabel *labTraslaY;
    QLineEdit *traslaY;
    QLabel *labTraslaZ;
    QLineEdit *traslaZ;
    QLabel *labRotaX;
    QLineEdit *rotaX;
    QLabel *labRotaY;
    QLineEdit *rotaY;
    QLabel *labRotaZ;
    QLineEdit *rotaZ;
    QLabel *labOffset;
    QLineEdit *timeOffset;
    QLabel *labFrequency;
    QLineEdit *frequency;//for interpolation to a given frequency
    //QLabel *labPCAIndex;
    //QLineEdit *pcaIndex;
    QCheckBox *gaussianNoiseCheck;
    QCheckBox *cosmicNoiseCheck;
    QLineEdit *gaussianNoiseDeviation;
    QLineEdit *cosmicNoiseDeviation;

    QRadioButton *freqMaxRadioButton;
    QRadioButton *freqMinRadioButton;
    QRadioButton *freqCustomizedRadioButton;

    //QCheckBox *Gaussian_Noise;
    //QCheckBox *Cosmic_Noise;
    QDialogButtonBox *buttonBox{};
    QPushButton *modifyButton{};
    //QPushButton *moreButton;
    QWidget *extension{};
    QPushButton *buttonOK;
    QPushButton *buttonCancel;
    DataDialogScalaTraslaRota* dialogModel{};
public slots:
    void onOK();
    void onCancel();
    void onPressCustomizedFrequency();
    void onPressMaxFrequency();
    void onPressMinFrequency();


 public:
    void setDataDialog(DataDialogScalaTraslaRota* aModel);
    DataDialogScalaTraslaRota* getDataDialog();

};

#endif // DIALOGSCALATRASLAROTA_H
