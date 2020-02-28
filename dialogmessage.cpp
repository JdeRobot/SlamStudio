
#include "dialogmessage.h"
#include "mainwindow.h"



DialogMessage::DialogMessage(QWidget *parent)
    : QDialog(parent)
{
    this->parent=(MainWindow *)parent;
    std::cout<< "constructor DialogMessage" <<std::endl;
    labelTitle = new QLabel(tr("Information"));

    labelMessage = new QLabel(tr("Please indicate transformation parameters ..."));



    QHBoxLayout *topLeftLayout = new QHBoxLayout;
    topLeftLayout->addWidget(labelTitle);


    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addLayout(topLeftLayout);
    leftLayout->addWidget(labelMessage);

    QGridLayout *mainLayout = new QGridLayout;
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);

    mainLayout->addLayout(leftLayout, 0, 0);

    mainLayout->setRowStretch(2, 1);

    setLayout(mainLayout);

    setWindowTitle(tr(" Information "));


    std::cout<< "FIN constructor DialogMessage" <<std::endl;
}

void DialogMessage::onOK()
{
   std::cout<< "FIN onOK" <<std::endl;
   this->close();
}

void DialogMessage::onCancel()
{
   std::cout<< "onCancel" <<std::endl;
   buttonCancel->hide();
   this->close();
}




