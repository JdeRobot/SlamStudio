#include <QtWidgets>
#include <iostream>
#include "mainwindow.h"
#include "dialogparameters.h"
//#include "datadialogparameters.h"
#include "configuration.h"

DialogParameters::DialogParameters()
= default;

DialogParameters::DialogParameters(QWidget *parent)
    : QDialog(parent)
{
    this->parent=(MainWindow *)parent;
    std::cout<< "constructor DialogScalaTraslaRota" <<std::endl;

    labMaxLines = new QLabel(tr("MAXLINES"));
    MaxLines = new QLineEdit;
    MaxLines->setText("50000");
    labStepOffset = new QLabel(tr("Step Offset"));
    stepOffset = new QLineEdit;
    stepOffset->setText("0.01");
    labWindowOffset = new QLabel(tr("Window Offset"));
    windowOffset = new QLineEdit;
    windowOffset->setText("3.0");
    labMessageCLEAN = new QLabel(tr("\nATTENTION: \n Be aware that if you pressed OK, \n datasets will be cleaned/deleted \n and you will have to reload and\n calculate the datasets again.\n"));

    buttonOK = new QPushButton(tr("&OK"));

    buttonCancel = new QPushButton(tr("&Cancel"));
    std::cout<< "constructor dialogScalaTraslaRota after Button Cancel" <<std::endl;

    connect(buttonOK, SIGNAL(clicked()), this, SLOT(onOK()));
    connect(buttonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));

    QVBoxLayout *leftLayout = new QVBoxLayout;
    //leftLayout->addLayout(topLeftLayout);

    leftLayout->addWidget(labMaxLines);
    leftLayout->addWidget(MaxLines);
    leftLayout->addWidget(labStepOffset);
    leftLayout->addWidget(stepOffset);
    leftLayout->addWidget(labWindowOffset);
    leftLayout->addWidget(windowOffset);
    leftLayout->addWidget(labMessageCLEAN);


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

    setWindowTitle(tr("Modify Configuration Parameters"));


}


void DialogParameters::onOK()
{
   std::cout<< "onOK" <<std::endl;
   //Take the data from screen
   double maxLines = MaxLines->text().toDouble();
   double offset_window = windowOffset->text().toDouble();
   double offset_step = stepOffset->text().toDouble();

   std::cout<< "onOK" <<std::endl;
   ((MainWindow*)(parent))->dataDialogParameters->updateData(maxLines,offset_step,offset_window);
   ((MainWindow*)(parent))->cleanDataSets();

   Configuration::setMaxLines(maxLines);
   Configuration::setStepOffset(offset_step);
   Configuration::setWindowOffset(offset_window);
   std::cout<< "FIN onOK" <<std::endl;
   this->close();
}

void DialogParameters::onCancel()
{
   std::cout<< "onCancel" <<std::endl;


   std::cout<< "FIN onCancel" <<std::endl;
   this->close();
}

void DialogParameters::setDataDialog(DataDialogParameters* aModel){

std::cout<<"aModel->getMAXLINES()"<<aModel->getMAXLINES()<<std::endl;
std::cout<<"aModel->getStepOffset()"<<aModel->getStepOffset()<<std::endl;
std::cout<<"aModel->getOffsetWindow()"<<aModel->getOffsetWindow()<<std::endl;
MaxLines->setText(QString::number(aModel->getMAXLINES(),'f',6));
stepOffset->setText(QString::number(aModel->getStepOffset(),'f',6));
windowOffset->setText(QString::number(aModel->getOffsetWindow(),'f',6));


}

