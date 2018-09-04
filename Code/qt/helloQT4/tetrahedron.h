
#ifndef TETRAHEDRON_H
#define TETRAHEDRON_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include "Eigen/Dense"
class Tetrahedron : public QOpenGLWidget, protected QOpenGLFunctions
{
Q_OBJECT
public:
Tetrahedron(QWidget *parent = 0);
void setDataView(Eigen::MatrixXd dataModel);
void setContaminatedDataView(Eigen::MatrixXd dataModelContaminated);
void setScala(double X, double Y, double Z);
void setTrasla(double X, double Y, double Z);

protected:
void initializeGL();
void resizeGL(int width, int height);
void paintGL();
void mousePressEvent(QMouseEvent *event);
void mouseMoveEvent(QMouseEvent *event);
void mouseDoubleClickEvent(QMouseEvent *event);
void wheelEvent(QWheelEvent *event);
double dataGTx[15000]= {0.0};
double dataGTy[15000]= {0.0};
double dataGTz[15000]= {0.0};
double dataContaminatedx[15000]= {0.0};
double dataContaminatedy[15000]= {0.0};
double dataContaminatedz[15000]= {0.0};



private:
int contLin1=0;
int contLin2=0;//number of lines of contaminated sequence
double scalaX=0.0;
double scalaY=0.0;
double scalaZ=0.0;
double traslaX=0.0;
double traslaY=0.0;
double traslaZ=0.0;

void draw();
QWidget *myParent;

int faceAtPosition(const QPoint &pos);
GLfloat rotationX;
GLfloat rotationY;
GLfloat rotationZ;
GLfloat m_scale;
QColor faceColors[4];
QPoint lastPos;

};
#endif // TETRAHEDRON_H
