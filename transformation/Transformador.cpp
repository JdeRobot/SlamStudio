
#include "Transformador.h"
using namespace Eigen;

Matrix3d Transformador::getMatRot_toQuaternion()
{
  return this->matRot_toQuaternion;
}

void Transformador::setMatRot_toQuaternion(Matrix3d aMatrix)
{
  //matRot_toQuaternion= aMatrix;
  matRot_toQuaternion = aMatrix.block(0, 0, 3, 3);
}
void Transformador::displayMatriz(double matriz[4][4], int matriz_rows, int matriz_cols)
{

  std::cout << "-----------------------------------------------display Matriz Rotacion-traslacion\n\n";
  for (int i = 0; i < matriz_rows; i++)
  {
    for (int j = 0; j < matriz_cols; j++)
    {
      std::cout << "\t" << matriz[i][j];
    }
    std::cout << "\n\n";
  }

}

void Transformador::displayMatrizRotTrasla()
{
  displayMatriz(matRotTrasla, 4, 4);
}

void Transformador::createMatRotTraslaEscala(float rotaX, float rotaY, float rotaZ, Point3D aPoint3D, Point3D aScala)
{
  createMatRot(rotaX, rotaY, rotaZ);
  setTraslacion(aPoint3D);//set traslation
  double matScala[4][4];

  matScala[0][0] = aScala.getX();
  matScala[0][1] = 0.0;
  matScala[0][2] = 0.0;
  matScala[0][3] = 0.0;

  matScala[1][0] = 0.0;
  matScala[1][1] = aScala.getY();
  matScala[1][2] = 0.0;
  matScala[1][3] = 0.0;

  matScala[2][0] = 0.0;
  matScala[2][1] = 0.0;
  matScala[2][2] = aScala.getZ();
  matScala[2][3] = 0.0;

  matScala[3][0] = 0.0;
  matScala[3][1] = 0.0;
  matScala[3][2] = 0.0;
  matScala[3][3] = 1.0;

  multiplicaMatrizPorMatriz(matRotTrasla, matScala, 4, 4, 4, 4);

}

void Transformador::createMatRot(float radX, float radY, float radZ)
{

  radX = radX * 180.0f / (float) M_PI;
  radY = radY * 180.0f / (float) M_PI;
  radZ = radZ * 180.0f / (float) M_PI;
  Matrix4d Rx, Ry, Rz, rotMatrix;
  Rx << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Ry << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Rz << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  matRotTrasla[0][0] = 1.0;
  matRotTrasla[0][1] = 0.0;
  matRotTrasla[0][2] = 0.0;
  matRotTrasla[0][3] = 0.0;

  matRotTrasla[1][0] = 0.0;
  matRotTrasla[1][1] = 1.0;
  matRotTrasla[1][2] = 0.0;
  matRotTrasla[1][3] = 0.0;

  matRotTrasla[2][0] = 0.0;
  matRotTrasla[2][1] = 0.0;
  matRotTrasla[2][2] = 1.0;
  matRotTrasla[2][3] = 0.0;

  matRotTrasla[3][0] = 0.0;
  matRotTrasla[3][1] = 0.0;
  matRotTrasla[3][2] = 0.0;
  matRotTrasla[3][3] = 1.0;

  if (radX != 0)
  {

    Rx << 1, 0, 0, 0,
        0, cosf(radX), sinf(radX), 0,
        0, -sinf(radX), cosf(radX), 0,
        0, 0, 0, 1;

    matRotTrasla[1][0] = 0.0;
    matRotTrasla[1][1] = cosf(radX);
    matRotTrasla[1][2] = sinf(radX);
    matRotTrasla[1][3] = 0.0;

    matRotTrasla[2][0] = 0.0;
    matRotTrasla[2][1] = -sinf(radX);
    matRotTrasla[2][2] = cosf(radX);
    matRotTrasla[2][3] = 0.0;

  }
  if (radY != 0)
  {
    Ry << cosf(radY), 0, -sinf(radY), 0,
        0, 1, 0, 0,
        sinf(radY), 0, cosf(radY), 0,
        0, 0, 0, 1;

    matRotTrasla[0][0] = cosf(radY);
    matRotTrasla[0][1] = 0.0;
    matRotTrasla[0][2] = -sinf(radY);
    matRotTrasla[0][3] = 0.0;

    matRotTrasla[2][0] = sinf(radY);
    matRotTrasla[2][1] = 0.0;
    matRotTrasla[2][2] = cosf(radY);
    matRotTrasla[2][3] = 0.0;

  }
  if (radZ != 0)
  {
    Rz << cosf(radZ), sinf(radZ), 0, 0,
        -sinf(radZ), cosf(radZ), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    matRotTrasla[0][0] = cosf(radZ);
    matRotTrasla[0][1] = sinf(radZ);
    matRotTrasla[0][2] = 0.0;
    matRotTrasla[0][3] = 0.0;

    matRotTrasla[1][0] = -sinf(radZ);
    matRotTrasla[1][1] = cosf(radZ);
    matRotTrasla[1][2] = 0.0;
    matRotTrasla[1][3] = 0.0;

  }

  rotMatrix = Rx * Ry * Rz;
  matRotTrasla[0][0] = rotMatrix(0, 0);
  matRotTrasla[0][1] = rotMatrix(0, 1);
  matRotTrasla[0][2] = rotMatrix(0, 2);
  matRotTrasla[0][3] = rotMatrix(0, 3);
  matRotTrasla[1][0] = rotMatrix(1, 0);
  matRotTrasla[1][1] = rotMatrix(1, 1);
  matRotTrasla[1][2] = rotMatrix(1, 2);
  matRotTrasla[1][3] = rotMatrix(1, 3);
  matRotTrasla[2][0] = rotMatrix(2, 0);
  matRotTrasla[2][1] = rotMatrix(2, 1);
  matRotTrasla[2][2] = rotMatrix(2, 2);
  matRotTrasla[2][3] = rotMatrix(2, 3);
  matRotTrasla[3][0] = rotMatrix(3, 0);
  matRotTrasla[3][1] = rotMatrix(3, 1);
  matRotTrasla[3][2] = rotMatrix(3, 2);
  matRotTrasla[3][3] = rotMatrix(3, 3);

  //Matrix3d matRot_toQuaternion;
  Matrix3d aMatrix;
  aMatrix << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2),
      rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2),
      rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2);
  this->setMatRot_toQuaternion(aMatrix);

}

Point3D Transformador::multiplicaMatrizPunto(double unPunto[4][1], int colsUnPunto, int rowsUnPunto)
{
//void Transformador::multiplicaMatrizPunto (double matriz [4][4], Point3D unPunto3D, int colsMatriz, int rowsMatriz, int colsUnPunto , int rowsUnPunto){


  Point3D aPoint3D;

  int matriz_cols = 4;
  int unPunto_rows = rowsUnPunto;
  int matriz_rows = 4;
  int unPunto_cols = colsUnPunto;

  if (matriz_cols == unPunto_rows)
  {

    //initialize the final array
    matriz_rows = 4;
    unPunto_cols = 1;
    //float c[][1]={0.0};

    int i, j, k;
    for (i = 0; i < matriz_rows; i++)
    {

      for (j = 0; j < unPunto_cols; j++)
      {

        newPunto[i][j] = 0;

        for (k = 0; k < unPunto_rows; k++)
        {

          newPunto[i][j] += matRotTrasla[i][k] * unPunto[k][j];

        }
      }
    }
    aPoint3D.setXYZ(newPunto[0][0], newPunto[1][0], newPunto[2][0]);

  } else
  {
    std::cout << "Multiplication of the two array is not possible";
  }
  return aPoint3D;

}
Point3D Transformador::multiplicaMatrizPunto(double matriz[4][4],
                                             double unPunto[4][1],
                                             int colsMatriz,
                                             int rowsMatriz,
                                             int colsUnPunto,
                                             int rowsUnPunto)
{
//void Transformador::multiplicaMatrizPunto (double matriz [4][4], Point3D unPunto3D, int colsMatriz, int rowsMatriz, int colsUnPunto , int rowsUnPunto){


  //Point3D aPoint3D = new Point3D();
  Point3D aPoint3D;

  int matriz_cols = colsMatriz;
  int unPunto_rows = rowsUnPunto;
  int matriz_rows = rowsMatriz;
  int unPunto_cols = colsUnPunto;

  if (matriz_cols == unPunto_rows)
  {
    std::cout
        << "\nThe number of cols in the first array is same as the number of rows in the second array, \nThe multiplication is possible\n";
    //initialize the final array
    matriz_rows = 4;
    unPunto_cols = 1;
    //float c[][1]={0.0};

    int i, j, k;
    for (i = 0; i < matriz_rows; i++)
    {

      for (j = 0; j < unPunto_cols; j++)
      {

        newPunto[i][j] = 0;

        for (k = 0; k < unPunto_rows; k++)
        {

          newPunto[i][j] += matriz[i][k] * unPunto[k][j];

        }
      }
    }
    aPoint3D.setXYZ(newPunto[0][0], newPunto[1][0], newPunto[2][0]);

  } else
  {
    std::cout << "Multiplication of the two array is not possible";
  }
  return aPoint3D;

}

void Transformador::multiplicaMatrizPorMatriz(double matrizA[4][4],
                                              double matrizB[4][4],
                                              int colsMatrizA,
                                              int rowsMatrizA,
                                              int colsMatrizB,
                                              int rowsMatrizB)
{
  int a_cols = colsMatrizA;
  int b_rows = rowsMatrizB;
  int a_rows = rowsMatrizA;
  int b_cols = colsMatrizB;

  if (a_cols == b_rows)
  {
    //initialize the final array
    double c[a_rows][b_cols];
    int i, j, k;
    for (i = 0; i < a_rows; i++)
    {
      for (j = 0; j < b_cols; j++)
      {
        c[i][j] = 0;
        for (k = 0; k < b_rows; k++)
        {
          c[i][j] += matrizA[i][k] * matrizB[k][j];
        }
      }
    }

    for (i = 0; i < a_rows; i++)
    {
      for (j = 0; j < b_cols; j++)
      {
        matRotTrasla[i][j] = c[i][j];
      }
    }
  } else
  {
    std::cout << "Multiplication of the two array is not possible";
  }

}

void Transformador::setTraslacion(Point3D aPoint3D)
{
  matRotTrasla[0][3] = aPoint3D.getX();
  matRotTrasla[1][3] = aPoint3D.getY();
  matRotTrasla[2][3] = aPoint3D.getZ();
}
void Transformador::setPoint3D(Point3D aPoint3D)
{
  this->myPoint3D.setXYZ(aPoint3D.getX(), aPoint3D.getY(), aPoint3D.getZ());

}
void Transformador::setInitTime(double initValue)
{
  initTime = initValue;
}
double Transformador::getInitTime()
{
  return initTime;
}
void Transformador::setOffset(double myOffset)
{
  offset = myOffset;
}
double Transformador::getOffset()
{
  return offset;
}

void Transformador::setFrequency(double myFrequency)
{
  frequency = myFrequency;
}

double Transformador::getFrequency()
{
  return frequency;
}
double Transformador::generateGaussianNoise(double mu, double sigma, int CNoise)
{
  static const double epsilon = std::numeric_limits<double>::min();
  static const double two_pi = 2.0 * 3.14159265358979323846;

  double z1;
  bool generate;
  generate = !generate;

  if (!generate)
    return z1 * sigma + mu;

  double u1, u2;
  do
  {
    u1 = rand() * (1.0 / RAND_MAX);
    u2 = rand() * (1.0 / RAND_MAX);
  } while (u1 <= epsilon);

  int cosmicNoise = 1 + (int) (300.0 * rand() / (RAND_MAX + 1.0));

  //std::cout<<"cosmicNoise="<<cosmicNoise<<"\n";

  int probabilityToApplyNoise = 290;
  if (CNoise == 1)
  {// if Cosmic Noise ==1 , then generate Cosmic Noise
    if (cosmicNoise % (probabilityToApplyNoise + 1) != probabilityToApplyNoise)
    {

      cosmicNoise = 0;
      //To apply cosmicNoise, we need that cosmicNoise % (probability+1) must be equal to probability+1
      //for instance, if probabilityTAN=5
      //then we need that cosmicNoise % 6 == 5, otherwise cosmicNoise=0

    } else
    {
      this->contCosmicNoise++;
      if (this->contCosmicNoise % 2 == 0)
        cosmicNoise = -cosmicNoise;
      std::cout << "##############################################cosmicNoise will be applied " << cosmicNoise
                << " counter= " << this->contCosmicNoise << " \n";

    }
  }
  double z0;
  z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
  z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
  if (CNoise == 1)
  {
    //return z0 * sigma + mu + cosmicNoise/12;
    //return z0 * sigma + mu + cosmicNoise/120;
    return z0 * sigma + mu + cosmicNoise / 100;
  } else
  {
    return z0 * sigma + mu;
  }
  //return z0 * sigma + mu + cosmicNoise/200 ;
  //return z0 * sigma + mu + (cosmicNoise % 10 + 2 );
  //return z0 * sigma + mu;
}

void Transformador::createContaminatedSequence(char *inputFileName,
                                               char *outputFileName,
                                               Point3D traslacion,
                                               Point3D escala,
                                               double rotaX,
                                               double rotaY,
                                               double rotaZ,
                                               int GNoise,
                                               int CNoise,
                                               double offset,
                                               int freqType,
                                               double frequency,
                                               double gnoise_value)
{
  // This function modify an input dataset or sequence, and create another new dataset or sequence modified
  // traslacion: is the traslation value (X,Y,Z)
  // escala    : is the scale value (X,Y,Z)
  // rotaX : is the angle value for Rotation on Axis X
  // rotaY : is the angle value for Rotation on Axis Y
  // rotaZ : is the angle value for Rotation on Axis Z
  // GNoise    : boolean value to indicate if Gaussian Noise must be applied to create the new modified dataset
  // CNoise    : boolean value to indicate if Cosmic Noise must be applied to create the new modified dataset
  // offset    : Indicates time offset
  std::ofstream out(outputFileName);
  out << std::setprecision(6) << std::fixed;

  int contLin = 0;
  Transformador myTransformador;
  myTransformador.setFrequency(0);
  myTransformador.setOffset(offset);
  myTransformador.setInitTime(-1);
  std::ifstream infile(inputFileName);
  std::string line;
  infile >> std::setprecision(6) >> std::fixed;

  myTransformador.createMatRotTraslaEscala(rotaX, rotaY, rotaZ, traslacion, escala);
  this->setMatRot_toQuaternion(myTransformador.getMatRot_toQuaternion());

  double gnoise; // variable to calculate Gaussian Noise
  int chooseXYZ = 0;
  int contTime = 0;
  double anOffset = myTransformador.getOffset();
  int MAX_ROWS = 50000;
  MatrixXd newDataSet(MAX_ROWS, 8);
  //Generate quaternion from rotation matrix
  Quaterniond q(matRot_toQuaternion);

  q.normalize();
  while (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4)
  {

    if (myTransformador.getInitTime() < 0)
    {

      myTransformador.setInitTime(timestamp + anOffset); //Take the first timestamp to init the time
      contTime = myTransformador.getInitTime();

    }

    myPoint3D.setX(rx);
    myPoint3D.setY(ry);
    myPoint3D.setZ(rz);

    myTransformador.setPoint3D(myPoint3D);

    punto[0][0] = myPoint3D.getX();
    punto[1][0] = myPoint3D.getY();
    punto[2][0] = myPoint3D.getZ();
    punto[3][0] = 1;

    myPoint3D = myTransformador.multiplicaMatrizPunto(punto, 1, 4);

    //modify the Point3D adding Gaussian Noise

    if (GNoise == 1)
    {
      gnoise = myTransformador.generateGaussianNoise(0, gnoise_value, CNoise);//mean , deviation, cosmicNoise
      chooseXYZ = long(abs(gnoise * 10000)) % 3;//
      switch (chooseXYZ)
      {
        case 0: // Adding Gaussian noise to coordenate X
          myPoint3D.setX(myPoint3D.getX() + gnoise);
          break;
        case 1: // Adding Gaussian noise to coordenate Y
          myPoint3D.setY(myPoint3D.getY() + gnoise);
          break;
        case 2: // Adding Gaussian noise to coordenate Z
          myPoint3D.setZ(myPoint3D.getZ() + gnoise);
          break;
        default:std::cout << "Error calculating mod 3" << "\n";
          break;
      }
    }

    //Rotate quaternion pData
    Quaterniond pData(q1, q2, q3, q4);
    Quaterniond rotatedP = q * pData * q.inverse();

    if (myTransformador.getFrequency() > 0)
    {

      out << contTime++ * myTransformador.getFrequency() << " " << myPoint3D.getX() << " " << myPoint3D.getY() << " "
          << myPoint3D.getZ() << " " << rotatedP.w() << " " << rotatedP.x() << " " << rotatedP.y() << " "
          << rotatedP.z() << std::endl;
    } else
    {
      double newTimeStamp = timestamp + anOffset;
      long intTimeStamp = newTimeStamp * 1000;
      newTimeStamp = intTimeStamp / 1000.0;
      newDataSet.row(contLin) << newTimeStamp, myPoint3D.getX(), myPoint3D.getY(), myPoint3D.getZ(), rotatedP
          .w(), rotatedP.x(), rotatedP.y(), rotatedP.z();
      contLin++;

    }

  }
  infile.close();

  // Now is time to interpolate the output serie
  Interpolator myInterpolator;
  MatrixXd newDataSet2(contLin, 8);
  newDataSet2 = newDataSet.block(0, 0, contLin, 8);
  if (freqType == 0) //max freq
  {
    myInterpolator.interpolateSerieToFrequency2(frequency, newDataSet2);

  } else if (freqType == 1) // min freq
  {

  } else if(freqType ==2) // custom freq
  {

  }
  for (int i = 0; i < newDataSet2.rows(); i++)
  {
    std::cout << newDataSet2.rows() << " " << i << "   " << newDataSet2.row(i) << ";" << newDataSet2.row(i) << "\n";

    out << newDataSet2.row(i)(0) << " " << newDataSet2.row(i)(1) << " " << newDataSet2.row(i)(2) << " "
        << newDataSet2.row(i)(3) << " " << newDataSet2.row(i)(4) << " " << newDataSet2.row(i)(5) << " "
        << newDataSet2.row(i)(6) << " " << newDataSet2.row(i)(7) << std::endl;
  }
  out.close();
  //Need to store the new interpolated dataSet


}

