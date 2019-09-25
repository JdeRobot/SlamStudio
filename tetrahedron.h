/*This class draw every point on 3d world
 *
 *
*/
#ifndef TETRAHEDRON_H
#define TETRAHEDRON_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include "Eigen/Dense"
#include <QColorDialog>
#include <QMouseEvent>
#include <GL/glu.h>
#include <GL/glut.h>
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <iostream>

class Tetrahedron : public QOpenGLWidget, protected QOpenGLFunctions
{
Q_OBJECT
public:
Tetrahedron(QWidget *parent = 0);
void setDataView(Eigen::MatrixXd dataModel);
void setContaminatedDataView(Eigen::MatrixXd dataModelContaminated);
void setEstimatedDataView(Eigen::MatrixXd dataModelEstimated);
void setScala(double X, double Y, double Z);
void setTrasla(double X, double Y, double Z);
void setDots();
void setLines();
void setViewJustEstimated();

protected:
void initializeGL();
void resizeGL(int width, int height);
void paintGL();
void mousePressEvent(QMouseEvent *event);
void mouseMoveEvent(QMouseEvent *event);
void mouseDoubleClickEvent(QMouseEvent *event);
void wheelEvent(QWheelEvent *event);

// Data Structure for data model: dataGT , data Contaminated, dataEstimated
// dataGTxyz is the original data
double dataGTx[15000]= {0.0};
double dataGTy[15000]= {0.0};
double dataGTz[15000]= {0.0};
// dataContaminatedxyz is the data modified with rotation , scala, traslation
double dataContaminatedx[15000]= {0.0};
double dataContaminatedy[15000]= {0.0};
double dataContaminatedz[15000]= {0.0};
// dataEstimated is the data estimated by register module
double dataEstimatedx[15000]= {0.0};
double dataEstimatedy[15000]= {0.0};
double dataEstimatedz[15000]= {0.0};
bool dots = true;
bool lines = false;
bool viewJustEstimated = false; //Show just estimated dataset


private:
int contLin1=0;
int contLin2=0;//number of lines of contaminated sequence
int contLin3=0;//number of lines of contaminated sequence
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
