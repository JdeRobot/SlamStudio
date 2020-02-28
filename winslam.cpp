#include "winslam.h"


Winslam::Winslam(MainWindow *mw)
        : mainWindow(mw)
{
    tetrahedron = new Tetrahedron;

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *container = new QHBoxLayout;

    container->addWidget(tetrahedron);

    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);

    setLayout(mainLayout);

    setWindowTitle(tr("SLAMTestBed"));
}

QSlider *Winslam::createSlider()
{
    QSlider *slider = new QSlider(Qt::Vertical);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    return slider;
}

void Winslam::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else
        QWidget::keyPressEvent(e);
}

void Winslam::setDataView(Eigen::MatrixXd dataModel){
    tetrahedron->setDataView(dataModel);
}

void Winslam::setContaminatedDataView(Eigen::MatrixXd dataModelContaminated){
    tetrahedron->setContaminatedDataView(dataModelContaminated);
}

void Winslam::setEstimatedDataView(Eigen::MatrixXd dataModelEstimated){
    tetrahedron->setEstimatedDataView(dataModelEstimated);
}

void Winslam::setScala(double X, double Y, double Z){
    std::cout<< "WinSlam.setScala" <<std::endl;
    tetrahedron->setScala(X,Y,Z);
}

void Winslam::setTrasla(double X, double Y, double Z){
    tetrahedron->setTrasla(X,Y,Z);
}
void Winslam::setDots(){
    tetrahedron->setDots();
}

void Winslam::setLines(){
    tetrahedron->setLines();
}
void Winslam::setViewJustEstimated(){
    tetrahedron->setViewJustEstimated();
}
