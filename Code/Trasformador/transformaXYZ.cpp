#include <iostream>
#include <ctime>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <cmath>
#include <cstdlib>

#include <limits>


#include "Eigen/Dense"
using Eigen::MatrixXd;

#include "Point3D.h"
#include "Transformador.h"
//#include "RegisterModule.h"
//float traslacion [4] [4] = {0.0};

//Point3D myPoint3D (0.0,0.0,0.0);
Point3D myPoint3D ;

double timestamp,rx,ry,rz,q1,q2,q3,q4;

double matRot [4] [4] = {
    {0.0}
};

double matRotTrasla [4][4] = {

};

double traslacion [4] [4] = {

    {1.0, 0.0 , 0.0, 0.0},
    {0.0, 1.0 , 0.0, 0.0},
    {0.0, 0.0 , 1.0, 0.0},
    {0.0, 0.0 , 0.0, 1.0}

};

double identidad [4] [4] = {

    {1.0, 0.0 , 0.0, 0.0},
    {0.0, 1.0 , 0.0, 0.0},
    {0.0, 0.0 , 1.0, 0.0},
    {0.0, 0.0 , 0.0, 1.0}

};

double punto [4] [1] = {
    {10.0},
    {10.0},
    {10.0},
    {1.0}
};
//float punto [4] [1] = {0.0};
double newPunto [4]  [1] = {0.0};


void createMatRot ( char eje , float angulo) {

    float anguloRad = angulo * (float)M_PI / 180.0f;
    if (eje == 'X'){

        matRot[0][0]=1.0;
        matRot[0][1]=0.0;
        matRot[0][2]=0.0;
        matRot[0][3]=0.0;

        matRot[1][0]=0.0;
        matRot[1][1]=cosf(anguloRad);
        matRot[1][2]=sinf(anguloRad);
        matRot[1][3]=0.0;

        matRot[2][0]=0.0;
        matRot[2][1]=-sinf(anguloRad);
        matRot[2][2]=cosf(anguloRad);
        matRot[2][3]=0.0;

        matRot[3][0]=0.0;
        matRot[3][1]=0.0;
        matRot[3][2]=0.0;
        matRot[3][3]=1.0;




    }if (eje == 'Y'){

        matRot[0][0]=cosf(anguloRad);
        matRot[0][1]=0.0;
        matRot[0][2]=-sinf(anguloRad);
        matRot[0][3]=0.0;

        matRot[1][0]=0.0;
        matRot[1][1]=1.0;
        matRot[1][2]=0.0;
        matRot[1][3]=0.0;

        matRot[2][0]=sinf(anguloRad);
        matRot[2][1]=0.0;
        matRot[2][2]=cosf(anguloRad);
        matRot[2][3]=0.0;

        matRot[3][0]=0.0;
        matRot[3][1]=0.0;
        matRot[3][2]=0.0;
        matRot[3][3]=1.0;

    } else if (eje == 'Z') {

        matRot[0][0]=cosf(anguloRad);
        matRot[0][1]=sinf(anguloRad);
        matRot[0][2]=0.0;
        matRot[0][3]=0.0;

        matRot[1][0]=-sinf(anguloRad);
        matRot[1][1]=cosf(anguloRad);
        matRot[1][2]=0.0;
        matRot[1][3]=0.0;

        matRot[2][0]=0.0;
        matRot[2][1]=0.0;
        matRot[2][2]=1.0;
        matRot[2][3]=0.0;

        matRot[3][0]=0.0;
        matRot[3][1]=0.0;
        matRot[3][2]=0.0;
        matRot[3][3]=1.0;

    }
};

void displayPunto (double unPunto [4][1]) {
    std::cout<<"displayPunto\n";
    std::cout<<"\n"<<unPunto[0][0];
    std::cout<<"\n"<<unPunto[1][0];
    std::cout<<"\n"<<unPunto[2][0];
    std::cout<<"\n"<<unPunto[3][0];
};

void displayMatriz (double unaMatriz [4] [4] ) {
    std::cout<<"displayMatriz\n";


};

void multiplicaMatrizPunto (double matriz [4][4], double unPunto[4][1], int colsMatriz, int rowsMatriz, int colsUnPunto , int rowsUnPunto){


       int matriz_cols = colsMatriz;
       int unPunto_rows = rowsUnPunto;
       int matriz_rows  = rowsMatriz;
       int unPunto_cols = colsUnPunto;

        if (matriz_cols == unPunto_rows){
            std::cout << "\nThe number of cols in the first array is same as the number of rows in the second array, \nThe multiplication is possible\n";
            //initialize the final array
            matriz_rows=4;
            unPunto_cols=1;
            //float c[][1]={0.0};

            int i, j , k;
            for (i=0;i<matriz_rows; i++) {

                for (j=0;j<unPunto_cols;j++) {

                    newPunto[i][j]=0;

                    for (k=0;k<unPunto_rows;k++) {

                        newPunto[i][j] += matriz[i][k]*unPunto[k][j];

                    }
                }
            }

            std::cout << "\nthe final array has been created\n\n";
            std::cout << "time to see the results \n";


           /*for(i=0;i<matriz_rows;i++)
            {
                for(j=0;j<unPunto_cols;j++)
                {
                    std::cout<<"\t"<<c[i][j];
                }
                std::cout<<"\n\n";
            }
            */
        }
        else{
            std::cout << "Multiplication of the two array is not possible";
        }

};

void multiplicaMatrizPorMatriz (double matrizA [4][4], double matrizB[4][4], int colsMatrizA, int rowsMatrizA, int colsMatrizB , int rowsMatrizB){


    int a_cols=colsMatrizA;
    int b_rows=rowsMatrizB;
    int a_rows=rowsMatrizA;
    int b_cols=colsMatrizB;

    if (a_cols == b_rows){
       // std::cout << "\nThe number of cols in the first array is same as the number of rows in the second array, \nThe multiplication is possible\n";
        //initialize the final array
        //int c[a_rows][b_cols];
        int i, j,k;
        for (i=0;i<a_rows; i++){
            for (j=0;j<b_cols;j++){
                //c[i][j]=0;
                matRotTrasla[i][j]=0;
                for (k=0;k<b_rows;k++){
                    matRotTrasla[i][j] += matrizA[i][k]*matrizB[k][j];
                }
            }
        }
        //std::cout << "\nthe final array has been created\n\n";
        //std::cout << "time to see the results \n";

       for(i=0;i<a_rows;i++)
        {
            for(j=0;j<b_cols;j++)
            {
                std::cout<<"\t"<<matRotTrasla[i][j];
            }
            std::cout<<"\n\n";
        }
    }
    else{
        std::cout << "Multiplication of the two array is not possible";
    }

};


/*
double generateGaussianNoise(double mu, double sigma)
{
	static const double epsilon = std::numeric_limits<double>::min();
	static const double two_pi = 2.0*3.14159265358979323846;

	std::cout<<"------------------------------------------Inside generateGauusianNoise \n";
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
	 }
	while ( u1 <= epsilon );

	double z0;
	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);

	int cosmicNoise=1+(int) (10.0*rand()/(RAND_MAX+1.0));

	std::cout<<"cosmicNoise="<<cosmicNoise<<"\n";

	int probabilityToApplyNoise = 20;

	if ( cosmicNoise % (probabilityToApplyNoise+1) != probabilityToApplyNoise ) {

		cosmicNoise=0;
		//To apply cosmicNoise, we need that cosmicNoise % (probability+1) must be equal to probability+1
		//for instance, if probabilityTAN=5
		//then we need that cosmicNoise % 6 == 5, otherwise cosmicNoise=0

	}else {

		std::cout<<"cosmicNoise will be applied "<<cosmicNoise<<" \n";

	}
	return z0 * sigma + mu + cosmicNoise;
}
*/



int main( int argc, char** argv )
{


    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream out( "miEntradaModificada.txt" );
    out << std::setprecision(6) << std::fixed;


    //int borrame = sqrt(4);
    int contLin=0;
    Transformador myTransformador;
    myTransformador.setFrequency(0.3);
    myTransformador.setOffset(5);
    myTransformador.setInitTime(-1);
    //for (contLin=0;contLin <50; contLin++  ){
    // FOR READING FILE AND DRAW CURVE
    std::ifstream infile("miEntrada.txt");
    std::string line;
    infile >> std::setprecision(6) >> std::fixed;

    //createMatRot('Z',10*contLin);
    //multiplicaMatrizPorMatriz (matRot,traslacion,4,4,4,4);// el resultado quedara en matRotTrasla
    //Point3D traslacion(0,0,0.2*contLin);
    Point3D traslacion2;
    traslacion2.setXYZ(0,2,2);
    Point3D escala;
    traslacion2.setXYZ(1.5,5,1.5);
    //myTransformador.createMatRotTrasla('Z',10*contLin,traslacion);
    //myTransformador.createMatRotTrasla('Z',20,traslacion);
    //myTransformador.createMatRotTraslaEscala('Z',20,traslacion,escala);
    //myTransformador.displayMatrizRotTrasla();
    std::cout << "beFORE WHILE";
    double gnoise; // variable to calculate Gaussian Noise
    int chooseXYZ = 0;
    int contTime = 0;

    while ( infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4 ){

    	if (myTransformador.getInitTime() < 0){

    		double anOffset = myTransformador.getOffset();
    		myTransformador.setInitTime ( timestamp + anOffset ); //Take the first timestamp to init the time
    		contTime = myTransformador.getInitTime();


    	}
        //std::cout << "You said.... contLin"<< contLin << timestamp << " " << rx << " " << ry << " " << rz << " " << q1 << " " << q2 << " " << q3 << " " << q4 << "\n";
    	myPoint3D.setX(rx);
    	myPoint3D.setY(ry);
    	myPoint3D.setZ(rz);
        //std::cout << "antes setPoint3d\n";
    	myTransformador.setPoint3D(myPoint3D);
    	//std::cout<<"despues setPoint3d\n";
        punto[0][0]=myPoint3D.getX();
        punto[1][0]=myPoint3D.getY();
        punto[2][0]=myPoint3D.getZ();
        punto[3][0]=1;

        //multiplicaMatrizPunto (traslacion,punto,4,4,1,4);
        //myPoint3D=myTransformador.multiplicaMatrizPunto(matRotTrasla,punto,4,4,1,4);
        myPoint3D=myTransformador.multiplicaMatrizPunto(punto,1,4);

        //modify the Point3D adding Gaussian Noise
        //std::cout<<"-----------------antes generateGaussianNoise\n";
        gnoise=myTransformador.generateGaussianNoise(0,0.01);//mean , deviation
        //std::cout<<"-----------------despues generateGaussianNoise\n";
        chooseXYZ= abs(gnoise*10000) % 3;//
        switch (chooseXYZ){
        case 0: // Adding Gaussian noise to coordenate X
        	myPoint3D.setX(myPoint3D.getX()+gnoise);
            break;
        case 1: // Adding Gaussian noise to coordenate Y
        	myPoint3D.setY(myPoint3D.getY()+gnoise);
        	break;
        case 2: // Adding Gaussian noise to coordenate Z
        	myPoint3D.setZ(myPoint3D.getZ()+gnoise);
        	break;
        default:
        	std::cout<< "Error calculating mod 3"<<"\n";
        	break;
        }

        //out <<timestamp <<" "<< myPoint3D.getX() <<" "<< myPoint3D.getY() <<" "<< myPoint3D.getZ() <<" "<< q1 <<" "<< q2 <<" "<<q3 <<" "<< q4<<std::endl;
        out <<contTime++ * myTransformador.getFrequency()<<" "<< myPoint3D.getX() <<" "<< myPoint3D.getY() <<" "<< myPoint3D.getZ() <<" "<< q1 <<" "<< q2 <<" "<<q3 <<" "<< q4<<std::endl;

    }
    infile.close();
    std::cout << ">>>>>>>CIERRO INFILE\n";
    //}

    //multiplicaMatrizPunto (traslacion,punto,4,4,1,4);
    //displayPunto(newPunto);

    //multiplicaMatrizPorMatriz (matRot,traslacion,4,4,4,4);
    out.close();
    //infile.close();
    double noise,total;
    int g;
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    /*
    //Codigo para comprobar como esta distribuido el ruido gaussiano

     for (g=0;g<1000000;g++){
    	noise=generateGaussianNoise(0,1);
    	 std::cout <<"ruido= "<<noise<<"\n";
    	total= total + noise;
    }
    std::cout <<"numero de datos con ruido="<<g<<"\n";
    std::cout <<"La suma del ruido es= "<<total<<" y la media es"<< total / (g)<<"\n";
    */

    return 0;
}
