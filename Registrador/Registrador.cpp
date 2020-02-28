#include "Registrador.h"

using namespace std;

Registrador::Registrador()
{
  std::cout << "constructor por defecto" << std::endl;
}//

Matrix3d Registrador::getMatRot_toQuaternion()
{
  return matRot_toQuaternion;
}

void Registrador::applyRANSAC(MatrixXd A, MatrixXd B, MatrixXd &bestRotation, MatrixXd &bestTraslation)
{
  int iterations = 0;
  int maxIterations = 20;
  MatrixXd FinalRotation;
  Vector3d FinalTraslation;
  double bestError = 99999;
  float errorThreshold = 0.001;
  unsigned t0, t1;
  t0 = clock();
  int numRows;
  numRows = A.rows();
  int randMAX = 32767;
  int nvalores = int(numRows / 10);
  while (iterations < maxIterations)
  { // Comenzamos el bucle RANSANC
    iterations++;
    //std::cout << "B "<<B<<std::endl;

    // Select 30 points inliers,
    MatrixXd mInliers(nvalores, 3);
    MatrixXd mData(nvalores, 3);
    int i = 0;
    bool visitados[numRows] = {false};
    int aInlierIdx = 0;
    // Seleccionamos aleatoriamente los primeros supuestos inliers
    while (i < nvalores)
    {

      aInlierIdx = abs((1 + (int) (numRows * rand() / (randMAX + 1.0))) % numRows);

      visitados[aInlierIdx] = true;
      std::cout << i << " aInlierIdx " << aInlierIdx << std::endl;
      mInliers.row(i) = A.row(aInlierIdx);
      mData.row(i) = B.row(aInlierIdx);
      i++;

    }


    //std::cout << "mInliers "<<mInliers<<std::endl;

    //Registrador myRegistrador;
    MatrixXd rotationEstimated, traslationEstimated, inliersEstimated, second_xyz_aligned;

    this->rigid_transform_3D(mInliers, mData, rotationEstimated, traslationEstimated);
    this->applyTransformationsOverData(mInliers, inliersEstimated, rotationEstimated, traslationEstimated);

    MatrixXd mErr = inliersEstimated - mData;
    //        int numCols = model_aligned2.cols();

    //for (int w=0;w<numCols; w++){
    //	out<<w <<" "<< model_aligned2(0,w) <<" "<< model_aligned2(1,w) <<" "<< model_aligned2(2,w) <<" "<< w <<" "<< w <<" "<< w <<" "<< w<<std:: endl;
    //}
    //out.close();
    // calculo la media del error de la diferencia entre los inliers y los datos originales
    //std::cout << "mErr  \n"<< mErr <<std:: endl;
    // calculo el valor absoluto
    //std::cout << "mErr abs \n"<< mErr.cwiseAbs() <<std:: endl;
    // calculo la media por columnas

    Vector3d meanInliers = (mErr.cwiseAbs()).colwise().mean();
    //std::cout << "meanInliers \n"<< meanInliers <<std:: endl;

    //Una vez estimado el modelo con los primeros inliers (matriz Rotacion y traslacion) intentaré encontrar cuales del resto de puntos escajarian en el modelo

    for (int k = 0; k < numRows; k++)
    {

      if (!visitados[k])
      {
        Vector3d newInlier = A.row(k);
        MatrixXd newInlierTransformed;
        this->applyTransformationsOverData(newInlier.transpose(),
                                           newInlierTransformed,
                                           rotationEstimated,
                                           traslationEstimated);
        Vector3d dataVector = B.row(k);
        Vector3d vErrorInlier = (dataVector - newInlierTransformed.transpose()).cwiseAbs();
        if (vErrorInlier(0) < errorThreshold && vErrorInlier(1) < errorThreshold && vErrorInlier(2) < errorThreshold)
        {

          visitados[k] = true;

        }

      }
    }
    int contTotalInliers = 0;
    for (int c = 0; c < numRows; c++)
    {
      if (visitados[c]) contTotalInliers++;
    }
    MatrixXd mFinalInliers(contTotalInliers, 3);
    MatrixXd mFinalData(contTotalInliers, 3);
    MatrixXd mFinalEstimated;
    int idxFinalInlier = 0;
    for (int g = 0; g < numRows; g++)
    {

      if (visitados[g])
      {
        mFinalInliers.row(idxFinalInlier) = A.row(g);
        mFinalData.row(idxFinalInlier) = B.row(g);
        idxFinalInlier++;
      }
    }
    std::cout << "Num final Inliers " << idxFinalInlier << std::endl;
    this->rigid_transform_3D(mFinalInliers,
                             mFinalData,
                             rotationEstimated,
                             traslationEstimated); // Estimamos matriz rotacion traslacion
    this->applyTransformationsOverData(mFinalInliers, mFinalEstimated, rotationEstimated, traslationEstimated);

    mErr = mFinalEstimated - mFinalData;

    MatrixXd mErr2 = mErr.array().pow(2);
    double errorNumber = sqrt(mErr2.sum() / mErr.rows());

    std::cout << "RMSE= " << errorNumber << std::endl;

    if (bestError > errorNumber)
    {
      FinalRotation = rotationEstimated;
      FinalTraslation = traslationEstimated;
      bestError = errorNumber;
    }

  }
  t1 = clock();
  bestRotation = FinalRotation;
  bestTraslation = FinalTraslation;
  std::cout << "BestError= " << bestError << std::endl;
  std::cout << "FinalRotation= " << FinalRotation << std::endl;
  std::cout << "FinalTraslation= " << FinalTraslation << std::endl;


  // Una vez tenemos el mejor modelo, aplicamos a los puntos la mejor estimacion de la matriz de rotacion y traslacion a los
  //  y guardamos en un fichero la transformación resultante de dichos puntos

  //this->rigid_transform_3D(A,B,bestRotation,bestTraslation); // Estimate best rotation and Traslation Matrix
  //this->applyTransformationsOverData(A,bestDataEstimated,bestRotation,bestTraslation);


//    MatrixXd Final_xyz_aligned;
//    Final_xyz_aligned = FinalRotation * B.transpose() ;
//    MatrixXd mFinalTransMatrix (Final_xyz_aligned.rows(),Final_xyz_aligned.cols()),  Final_model_aligned2;
  //        for (int j=0; j< Final_xyz_aligned.cols(); j++ ) {

  //        	mFinalTransMatrix.col(j) << FinalTraslation(0), FinalTraslation(1), FinalTraslation(2);

  //        }
  //        std::cout << "mFinalTransMatrix \n"<< mFinalTransMatrix <<std:: endl;
  //        Final_model_aligned2=Final_xyz_aligned+mFinalTransMatrix;

  //        //std::cout << "Final_model_aligned2 \n"<< Final_model_aligned2 <<std:: endl;
  //        std::cout << "Final_model_aligned2.rows() \n"<< Final_model_aligned2.rows() <<std:: endl;
  //        std::cout << "Final_model_aligned2.cols() \n"<< Final_model_aligned2.cols() <<std:: endl;
  //        //MatrixXd alignment_error = model_aligned - data

  //    int numCols = Final_model_aligned2.cols();

  //    for (int w=0;w<numCols; w++){
  //    	out<<w <<" "<< Final_model_aligned2(0,w) <<" "<< Final_model_aligned2(1,w) <<" "<< Final_model_aligned2(2,w) <<" "<< w <<" "<< w <<" "<< w <<" "<< w<<std:: endl;
  //    }
  //    out.close();

  double time = (double(t1 - t0) / CLOCKS_PER_SEC);
  std::cout << "Execution Time: " << time << std::endl;

}

void Registrador::rigid_transform_3D_normalized(MatrixXd A, MatrixXd B, MatrixXd &R, MatrixXd &t)
{
  // compare size of matrix A y B
  //Matrix A and B have been transformed . A = A - centroidA  B = B - centroidB
  int N = 0;
  Matrix3d AA, BB, mCentroidA, mCentroidB;
  MatrixXd H, U, V, S;

  std::cout << "rigid_transform_3D 1" << std::endl;
  if (A.cols() * A.rows() == B.cols() * B.rows())
  { // the 2 arrays have similar size
    MatrixXd AA(A.rows(), A.cols()), BB(B.rows(), B.cols()), mCentroidA(A.rows(), A.cols()),
        mCentroidB(B.rows(), B.cols());

    N = A.rows(); // total points


    H = A.transpose() * B;

    JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);

    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();

    MatrixXd Vt = V.transpose();

    R = Vt.transpose() * U.transpose();
    std::cout << "R1: \n" << R << std::endl;

    if (R.determinant() < 0)
    {
      std::cout << "Detectadas reflections-------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>><" << std::endl;

      Vt.row(2) *= -1;
      R = Vt.transpose() * U.transpose();
      std::cout << "R2: \n" << R << std::endl;
    }

    Vector3d centroidA = A.colwise().mean();
    Vector3d centroidB = B.colwise().mean();

    t = R * centroidB - centroidA;
    std::cout << "t \n" << t << std::endl;

  }
}

void Registrador::rigid_transform_3D(MatrixXd A, MatrixXd B, MatrixXd &R, MatrixXd &t)
{

  int N = 0;
  Matrix3d AA, BB, mCentroidA, mCentroidB;
  MatrixXd H, U, V, S;

  std::cout << "rigid_transform_3D 1" << std::endl;
  if (A.cols() * A.rows() == B.cols() * B.rows())
  { // the 2 arrays have similar size
    MatrixXd AA(A.rows(), A.cols()), BB(B.rows(), B.cols()), mCentroidA(A.rows(), A.cols()),
        mCentroidB(B.rows(), B.cols());

    N = A.rows(); // total points

    double centroidAX = VectorXd(A.col(0)).mean();
    double centroidAY = VectorXd(A.col(1)).mean();
    double centroidAZ = VectorXd(A.col(2)).mean();

    Vector3d centroidA(centroidAX, centroidAY, centroidAZ);

    double centroidBX = VectorXd(B.col(0)).mean();
    double centroidBY = VectorXd(B.col(1)).mean();
    double centroidBZ = VectorXd(B.col(2)).mean();

    Vector3d centroidB(centroidBX, centroidBY, centroidBZ);

    for (int i = 0; i < N; i++)
    {
      //AA.row(i) = Vector3d(A.row(i)) - centroidA ;
      mCentroidA.row(i) << centroidAX, centroidAY, centroidAZ;

    }

    for (int j = 0; j < N; j++)
    {
      //BB.row(j) = Vector3d(B.row(j)) - centroidB ;
      mCentroidB.row(j) << centroidBX, centroidBY, centroidBZ;

    }
    AA = A - mCentroidA;
    BB = B - mCentroidB;

    std::cout << centroidA << std::endl;

    std::cout << centroidB << std::endl;

    H = AA.transpose() * BB;

    JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);

    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();

    MatrixXd Vt = V.transpose();

    R = Vt.transpose() * U.transpose();
    std::cout << "R1: \n" << R << std::endl;
    std::cout << "centroidA: " << centroidA << std::endl;
    std::cout << "centroidB: " << centroidB << std::endl;
    if (R.determinant() < 0)
    {
      std::cout << "Detected reflections-------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>><" << std::endl;

      Vt.row(2) *= -1;
      R = Vt.transpose() * U.transpose();
      std::cout << "R2: \n" << R << std::endl;
    }

    t = -R * centroidA + centroidB;
    std::cout << "t \n" << t << std::endl;

  }
}

void Registrador::applyTransformationsOverData(MatrixXd dataInitial,
                                               MatrixXd &dataEstimated,
                                               MatrixXd rotationEstimated,
                                               MatrixXd traslationEstimated)
{
  MatrixXd Final_xyz_aligned;
  std::cout << "Rotation_ estimated  " << rotationEstimated.rows() << "   " << rotationEstimated.cols() << std::endl;
  std::cout << "dataInitial  " << dataInitial.rows() << "   " << dataInitial.cols() << std::endl;
  Final_xyz_aligned = rotationEstimated * dataInitial.transpose();

  MatrixXd mFinalTransMatrix(Final_xyz_aligned.rows(), Final_xyz_aligned.cols()), Final_model_aligned2;
  for (int j = 0; j < Final_xyz_aligned.cols(); j++)
  {

    mFinalTransMatrix.col(j) << traslationEstimated(0), traslationEstimated(1), traslationEstimated(2);

  }

  Final_model_aligned2 = Final_xyz_aligned + mFinalTransMatrix;

  int numCols = Final_model_aligned2.cols();

  //the Estimated data will return on this MatrixXd
  dataEstimated = Final_model_aligned2.transpose();

  // Save estimated data to a file
  /*
  for (int w=0;w<numCols; w++){
      out<<w <<" "<< Final_model_aligned2(0,w) <<" "<< Final_model_aligned2(1,w) <<" "<< Final_model_aligned2(2,w) <<" "<< w <<" "<< w <<" "<< w <<" "<< w<<std::endl;
  }
  out.close();

  // Find the error
      MatrixXd err = Final_model_aligned2 - A;
      //std::cout << "err "<<std::endl<< err <<std::endl;
      MatrixXd err2 = err.array().pow(2);
      double errorNumber = sqrt(err2.sum()/err2.rows());

      //std::cout << "err2 "<< err2 <<std::endl;
      std::cout << "RMSE= "<< errorNumber <<std::endl;

      double time = (double(t1-t0)/CLOCKS_PER_SEC);
      std::cout << "ExecutdataEstimatedQuaternionion Time: " << time << std::endl;
  */
}

void Registrador::applyTransformationsOverQuaternion(MatrixXd dataInitialQuaternion,
                                                     MatrixXd &dataEstimatedQuaternion,
                                                     MatrixXd rotMatrix)
{
  // dataInitialQuaternion will have q1,q2,q3,q4
  // dataEstimated will have q1',q2',q3',q4' estimated
  // rotMatrix is rotationMatrixEstimated

  //Matrix3d matRot_toQuaternion; //important rotMatrix should be 3x3 to convert to a quaternion
  matRot_toQuaternion << rotMatrix(0, 0), rotMatrix(0, 1), rotMatrix(0, 2),
      rotMatrix(1, 0), rotMatrix(1, 1), rotMatrix(1, 2),
      rotMatrix(2, 0), rotMatrix(2, 1), rotMatrix(2, 2);

  Quaterniond q(matRot_toQuaternion);

  q.normalize(); //important quaternion from rotation matrix should be normalized
  dataEstimatedQuaternion = MatrixXd::Zero(dataInitialQuaternion.rows(), dataInitialQuaternion.cols());
  int rows = dataInitialQuaternion.rows();
  for (int i = 0; i < rows; i++)
  {
    //create quaternion p (p1 p2 p3 p4)
    Quaterniond pData(dataInitialQuaternion(i, 0),
                      dataInitialQuaternion(i, 1),
                      dataInitialQuaternion(i, 2),
                      dataInitialQuaternion(i, 3));
    Quaterniond rotatedP = q * pData * q.inverse();
    dataEstimatedQuaternion(i, 0) = rotatedP.w();
    dataEstimatedQuaternion(i, 1) = rotatedP.x();
    dataEstimatedQuaternion(i, 2) = rotatedP.y();
    dataEstimatedQuaternion(i, 3) = rotatedP.z();

  }

}

void Registrador::demoPresentaMatriz()
{

  MatrixXd m = MatrixXd::Random(3, 3);
  //m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
  std::cout << "m =" << m << std::endl;
  VectorXd v(3);
  v << 1, 2, 3;
  std::cout << "m * v =" << m * v << std::endl;

}

