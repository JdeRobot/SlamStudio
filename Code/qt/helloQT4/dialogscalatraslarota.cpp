#include <QtWidgets>
#include <iostream>
#include "dialogscalatraslarota.h"
#include "mainwindow.h"

DialogScalaTraslaRota::DialogScalaTraslaRota(QWidget *parent)
    : QDialog(parent)
{
    this->parent=(MainWindow *)parent;
    std::cout<< "constructor DialogScalaTraslaRota" <<std::endl;
    label = new QLabel(tr("Configure Modifications on data sequence:"));
    //lineEdit = new QLineEdit;
    //label->setBuddy(lineEdit);

    scaleCheckBox = new QCheckBox(tr("Scale"));
    scaleX = new QLineEdit;
    scaleX->setText("0");
    scaleY = new QLineEdit;
    scaleY->setText("0");
    scaleZ = new QLineEdit;
    scaleZ->setText("0");



    traslaCheckBox = new QCheckBox(tr("Traslation"));
    traslaX = new QLineEdit;
    traslaX->setText("0");
    traslaY = new QLineEdit;
    traslaY->setText("0");
    traslaZ = new QLineEdit;
    traslaZ->setText("0");

    rotaCheckBox = new QCheckBox(tr("Rotation"));
    rotaX = new QLineEdit;
    rotaX->setText("0");
    rotaY = new QLineEdit;
    rotaY->setText("0");
    rotaZ = new QLineEdit;
    rotaZ->setText("0");

    buttonOK = new QPushButton(tr("&OK"));

    buttonCancel = new QPushButton(tr("&Cancel"));

    connect(buttonOK, SIGNAL(clicked()), this, SLOT(onOK()));
    connect(buttonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));

    //fromStartCheckBox = new QCheckBox(tr("Search from &start"));
    //fromStartCheckBox->setChecked(true);

    //findButton = new QPushButton(tr("&Find"));
    //findButton->setDefault(true);

    //moreButton = new QPushButton(tr("&More"));
    //moreButton->setCheckable(true);
    //moreButton->setAutoDefault(false);


    //extension = new QWidget;

    //wholeWordsCheckBox = new QCheckBox(tr("&Whole words"));
    //backwardCheckBox = new QCheckBox(tr("Search &backward"));
    //searchSelectionCheckBox = new QCheckBox(tr("Search se&lection"));

    //buttonBox = new QDialogButtonBox(Qt::Vertical);
    //buttonBox->addButton(findButton, QDialogButtonBox::ActionRole);
    //buttonBox->addButton(moreButton, QDialogButtonBox::ActionRole);

    //connect(moreButton, &QAbstractButton::toggled, extension, &QWidget::setVisible);

    //QVBoxLayout *extensionLayout = new QVBoxLayout;
    //extensionLayout->setMargin(0);
    //extensionLayout->addWidget(wholeWordsCheckBox);
    //extensionLayout->addWidget(backwardCheckBox);
    //extensionLayout->addWidget(searchSelectionCheckBox);
    //extension->setLayout(extensionLayout);

    QHBoxLayout *topLeftLayout = new QHBoxLayout;
    topLeftLayout->addWidget(label);
    //topLeftLayout->addWidget(lineEdit);

    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addLayout(topLeftLayout);
    leftLayout->addWidget(scaleCheckBox);
    leftLayout->addWidget(scaleX);
    leftLayout->addWidget(scaleY);
    leftLayout->addWidget(scaleZ);
    leftLayout->addWidget(traslaCheckBox);
    leftLayout->addWidget(traslaX);
    leftLayout->addWidget(traslaY);
    leftLayout->addWidget(traslaZ);
    leftLayout->addWidget(rotaCheckBox);
    leftLayout->addWidget(rotaX);
    leftLayout->addWidget(rotaY);
    leftLayout->addWidget(rotaZ);


    QHBoxLayout *buttonLayout2 = new QHBoxLayout;
    buttonLayout2->addWidget(buttonOK);
    buttonLayout2->addWidget(buttonCancel);


    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);
    mainLayout->addLayout(leftLayout, 0, 0);
    mainLayout->addLayout(buttonLayout2, 1,0);
    //mainLayout->addWidget(buttonBox, 0, 1);
    //mainLayout->addWidget(extension, 1, 0, 1, 2);
    mainLayout->setRowStretch(2, 1);

    setLayout(mainLayout);

    setWindowTitle(tr("sequence manager"));
    //extension->hide();
}

void DialogScalaTraslaRota::onOK()
{
   std::cout<< "onOK" <<std::endl;
   double tx = traslaX->text().toDouble();
   double ty = traslaY->text().toDouble();
   double tz = traslaZ->text().toDouble();
   double sx = scaleX->text().toDouble();
   double sy = scaleY->text().toDouble();
   double sz = scaleZ->text().toDouble();
   double rx = rotaX->text().toDouble();
   double ry = rotaY->text().toDouble();
   double rz = rotaZ->text().toDouble();
   std::cout<< "onOK" <<std::endl;
   ((MainWindow*)(parent))->performModifySequence(sx,sy,sz,tx,ty,tz,rx,ry,rz);
   //((MainWindow*)(parent))->setTrasla(x,y,z);

   //((MainWindow*)(parent))->setTrasla(traslaX->text().toDouble(),traslaY->text().toDouble(),traslaZ->text().toDouble());
   //((Winslam*)(((MainWindow*)(parent))->myWinSlam))->setTrasla(traslaX,traslaY,traslaZ);
   std::cout<< "FIN onOK" <<std::endl;
   this->close();
}

void DialogScalaTraslaRota::onCancel()
{
   std::cout<< "onCancel" <<std::endl;
   buttonCancel->hide();
   this->close();
}
