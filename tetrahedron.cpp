#include "tetrahedron.h"

Tetrahedron::Tetrahedron(QWidget *parent)
: QOpenGLWidget(parent)
{
//setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));
m_scale=0.25;
rotationX = -21.0;
rotationY = -57.0;
rotationZ = 0.0;
faceColors[0] = Qt::red;
faceColors[1] = Qt::green;
faceColors[2] = Qt::blue;
faceColors[3] = Qt::yellow;
myParent= parent;

}

void Tetrahedron::initializeGL()
{
initializeOpenGLFunctions();
//qglClearColor(Qt::black);
glShadeModel(GL_FLAT);
glEnable(GL_DEPTH_TEST);
glEnable(GL_CULL_FACE);

}

void Tetrahedron::resizeGL(int width, int height)
{


    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
#ifdef QT_OPENGL_ES_1
    //glOrthof(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
    glOrthof(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
#else
    //glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
    //glOrtho(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
    glOrtho(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
#endif
    glMatrixMode(GL_MODELVIEW);

}

void Tetrahedron::paintGL(){
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//set background color to white
draw();
}

void Tetrahedron::draw()
{
    static const GLfloat P1[3] = {0.0, -1.0, +2.0};
    static const GLfloat P2[3] = {+1.73205081, -1.0, -1.0};
    static const GLfloat P3[3] = {-1.73205081, -1.0, -1.0 };
    static const GLfloat P4[3] = {0.0, +2.0, 0.0};
    static const GLfloat * const coords[4][3] = {
{ P1, P2, P3 }, { P1, P3, P4 }, { P1, P4, P2 }, { P2, P4, P3 }
};
glMatrixMode(GL_MODELVIEW);
glLoadIdentity();
glPointSize(2);   
glTranslatef(0.0, 0.0, -10.0);
glScalef(m_scale,m_scale,m_scale);
//glTranslatef(0.0, 0.0, -5.0);
glRotatef(rotationX, 1.0, 0.0, 0.0);
glRotatef(rotationY, 0.0, 1.0, 0.0);
glRotatef(rotationZ, 0.0, 0.0, 1.0);

/*
for (int i = 0; i < 4; ++i) {
    glLoadName(i);
    glBegin(GL_TRIANGLES);
    //qglColor(faceColors[i]);
    for (int j = 0; j < 3; ++j) {
        glVertex3f(coords[i][j][0], coords[i][j][1],coords[i][j][2]);
    }
    glEnd();
}
*/
// DRAW A  GRID
        glBegin(GL_LINES);
        for(int i=0;i<=100;i++) {
            if (i==0) { glColor3f(.6,.3,.3); } else { glColor3f(.35,.35,.35); };
            glVertex3f(i,0,0);
            glVertex3f(i,0,100);
            if (i==0) { glColor3f(.3,.3,.6); } else { glColor3f(.35,.35,.35); };
            glVertex3f(0,0,i);
            glVertex3f(100,0,i);

        };
        glColor3f(.3,1.0,.3);
        glVertex3f(0,0,0);
        glVertex3f(0,100,0);
        glEnd();

        //DRAW dataView. 3D points
        if (dots){
            glBegin(GL_POINTS);
            //glBegin(GL_LINES);
            if (!viewJustEstimated) {
                contLin1 --;
                for (int h=0; h < contLin1; h++){// Bucle for, important the < simbol instead of <=

                                        glVertex3f(dataGTx[h],dataGTy[h],dataGTz[h]); // LINEA QUE DIBUJA UNA CURVA
                                        glVertex3f(dataGTx[h+1],dataGTy[h+1],dataGTz[h+1]);
                }
                contLin1 ++;
            }
            glColor3f(.3,.3,1.0);
            //glBegin(GL_POINTS);
            contLin2 --;
            for (int j=0; j < contLin2; j++){// Bucle for, important the < simbol instead of <=

                                    glVertex3f(dataContaminatedx[j],dataContaminatedy[j],dataContaminatedz[j]); // LINEA QUE DIBUJA UNA CURVA
                                    glVertex3f(dataContaminatedx[j+1],dataContaminatedy[j+1],dataContaminatedz[j+1]);

            }
            contLin2 ++;
            glColor3f(1.0,.3,.3);
            contLin3 --;
            for (int k=0; k < contLin3; k++){// Bucle for, important the < simbol instead of <=

                                    glVertex3f(dataEstimatedx[k],dataEstimatedy[k],dataEstimatedz[k]); // LINEA QUE DIBUJA UNA CURVA
                                    glVertex3f(dataEstimatedx[k+1],dataEstimatedy[k+1],dataEstimatedz[k+1]);

            }
            contLin3 ++;
            glEnd();

        } else if (lines){


            glBegin(GL_LINES);
            if (!viewJustEstimated) {
                contLin1 --;
                for (int h=0; h < contLin1; h++){// Bucle for, important the < simbol instead of <=

                                        glVertex3f(dataGTx[h],dataGTy[h],dataGTz[h]); // LINEA QUE DIBUJA UNA CURVA
                                        glVertex3f(dataGTx[h+1],dataGTy[h+1],dataGTz[h+1]);
                }
                contLin1 ++;
            }
            glColor3f(.3,.3,1.0);


            //glBegin(GL_POINTS);
            contLin2 --;
            for (int j=0; j < contLin2; j++){// Bucle for, important the < simbol instead of <=

                                    glVertex3f(dataContaminatedx[j],dataContaminatedy[j],dataContaminatedz[j]); // LINEA QUE DIBUJA UNA CURVA
                                    glVertex3f(dataContaminatedx[j+1],dataContaminatedy[j+1],dataContaminatedz[j+1]);

            }
            contLin2 ++;

            glColor3f(1.0,.3,.3);
            contLin3 --;
            for (int k=0; k < contLin3; k++){// Bucle for, important the < simbol instead of <=

                                    glVertex3f(dataEstimatedx[k],dataEstimatedy[k],dataEstimatedz[k]); // LINEA QUE DIBUJA UNA CURVA
                                    glVertex3f(dataEstimatedx[k+1],dataEstimatedy[k+1],dataEstimatedz[k+1]);

            }
            contLin3 ++;
            glEnd();

        }

}

void Tetrahedron::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void Tetrahedron::mouseMoveEvent(QMouseEvent *event)
{
    GLfloat dx = GLfloat(event->x() - lastPos.x()) / width();GLfloat dy = GLfloat(event->y() - lastPos.y()) / height();
    if (event->buttons() & Qt::LeftButton) {
    rotationX += 180 * dy;
    rotationY += 180 * dx;
    //updateGL();
    update();
    } else if (event->buttons() & Qt::RightButton) {
    rotationX += 180 * dy;
    rotationZ += 180 * dx;
    //updateGL();
    update();
    }
    lastPos = event->pos();
}
void Tetrahedron::wheelEvent(QWheelEvent *event){
    std::cout <<"wheelEvent"<<"delta="<<event->delta()<<std::endl;
    if(event->delta() > 0) {
        // Zoom in
        m_scale +=0.0050;

    } else {
        // Zooming out
         m_scale -=0.0050;
    }
    std::cout <<"wheelEvent"<<"m_scale="<<m_scale<<std::endl;
    update();
}
void Tetrahedron::mouseDoubleClickEvent(QMouseEvent *event)
{
    //int face = faceAtPosition(event->pos());
    /*
    if (face != -1) {
        QColor color = QColorDialog::getColor(faceColors[face], this);
        if (color.isValid()) {
            faceColors[face] = color;
            //updateGL();
            update();
        }
    }
    */
}


void Tetrahedron::setDataView(Eigen::MatrixXd dataModel){
   for (int i=0;i<dataModel.rows();i++){
    dataGTx[i]=(dataModel.row(i))(0);
    dataGTy[i]=(dataModel.row(i))(1);
    dataGTz[i]=(dataModel.row(i))(2);

   }
   contLin1=dataModel.rows();

}

void Tetrahedron::setContaminatedDataView(Eigen::MatrixXd dataModelContaminated){
   for (int i=0;i<dataModelContaminated.rows();i++){
    dataContaminatedx[i]=(dataModelContaminated.row(i))(0);
    dataContaminatedy[i]=(dataModelContaminated.row(i))(1);
    dataContaminatedz[i]=(dataModelContaminated.row(i))(2);

   }
   contLin2=dataModelContaminated.rows();

}

void Tetrahedron::setEstimatedDataView(Eigen::MatrixXd dataModelEstimated){
   for (int i=0;i<dataModelEstimated.rows();i++){
    dataEstimatedx[i]=(dataModelEstimated.row(i))(0);
    dataEstimatedy[i]=(dataModelEstimated.row(i))(1);
    dataEstimatedz[i]=(dataModelEstimated.row(i))(2);

   }
   contLin3=dataModelEstimated.rows();
   update();
}

void Tetrahedron::setScala(double X, double Y, double Z){
    std::cout<< "tetrahedron.setScala" <<std::endl;
    scalaX=X;
    scalaY=Y;
    scalaZ=Z;
    std::cout<< "FIN tetrahedron.setScala" <<std::endl;

}

void Tetrahedron::setTrasla(double X, double Y, double Z){
    traslaX=X;
    traslaY=Y;
    traslaZ=Z;

    std::cout <<"Tetrahedron.setTrasla"<<"traslaX="<<traslaX<<"traslaY="<<traslaY<<"traslaZ="<<traslaZ;

}

void Tetrahedron::setDots(){
    dots=true;
    lines=false;
    update();
}

void Tetrahedron::setLines(){
    lines=true;
    dots=false;
    update();
}

void Tetrahedron::setViewJustEstimated(){
    viewJustEstimated=!viewJustEstimated;
    update();
}
