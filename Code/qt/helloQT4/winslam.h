#ifndef WINSLAM_H
#define WINSLAM_H
#include <QWidget>
#include "Eigen/Dense"

class QSlider;
class QPushButton;

class Tetrahedron;
class MainWindow;

class Winslam : public QWidget
{
    Q_OBJECT

public:
    Winslam(MainWindow *mw);
    void setDataView(Eigen::MatrixXd dataModel);
    void setContaminatedDataView(Eigen::MatrixXd dataModelContaminated);
    void setEstimatedDataView(Eigen::MatrixXd dataModelEstimated);
    void setScala(double X, double Y, double Z);
    void setTrasla(double X, double Y, double Z);
    void setDots();
    void setLines();
protected:
    void keyPressEvent(QKeyEvent *event) override;


private slots:
    //void dockUndock();

private:
    QSlider *createSlider();

    Tetrahedron *tetrahedron;

    /*QSlider *xSlider;
    QSlider *ySlider;
    QSlider *zSlider;
    QPushButton *dockBtn;
    */
    MainWindow *mainWindow;
    double arx[15000]= {0.0};
    double ary[15000]= {0.0};
    double arz[15000]= {0.0};
    double scalaX = 0.0;
    double scalaY = 0.0;
    double scalaZ = 0.0;
    double traslaX = 0.0;
    double traslaY = 0.0;
    double traslaZ = 0.0;


};
#endif // WINSLAM_H
