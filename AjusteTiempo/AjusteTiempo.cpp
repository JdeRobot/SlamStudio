

#include "AjusteTiempo.h"
using namespace Eigen;
AjusteTiempo::AjusteTiempo(){
	std::cout<< "constructor por defecto AjusteTiempo" <<std::endl;
}

void calculateDistanceForDataSet3D( MatrixXd dataSet, Matrix2Xd& time_Distance3D ){
    int rows = dataSet.rows();
    double x=0, y=0, z=0;
    time_Distance3D = Matrix2Xd::Zero(dataSet.rows(),2);
    for (int i=0; i < rows; i++){
        x=dataSet.row(i)(1);
        y= dataSet.row(i)(2);
        z= dataSet.row(i)(3);
        // the Distance to the origin (0,0,0) of a 3d point (x,y, z)
        // should be calculated like sqrt( (0-x)^2 + (0-y)^2 + (0-z)^2)
        time_Distance3D.row(i)<< dataSet.row(i)(0), sqrt( x*x + y*y + z*z );
    }
}

void AjusteTiempo::calcularAutocorrelacion(int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2)

{
	// offset: define la separación de la segunda serie que sera una serie artificial. La segunda serie S2 = S1 + offset
	// intervalo: define la amplitud de la ventana desde la que buscaremos la correlación
	// maxLine : numero máximo de líneas


    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

	std::cout <<"antes de declarar la variable"<<std::endl;
	//int maxLine = 30;
	MatrixXd readingA (maxLine,4);//size is big enough to store an unknown number of data rows from file
	MatrixXd readingB (maxLine,4);
	VectorXd vTiempoA (maxLine), vSerieA (maxLine) ;
	VectorXd vTiempoB (maxLine), vSerieB (maxLine) ;
	//read a file
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	double mediaTiempoA, mediaTiempoB, mediaSerieA, mediaSerieB;
	std::cout <<"antes de leer el archivo"<<std::endl;
	//int miOffset=5;
    double miOffset = offset;
    double newContLin = 0;
    std::cout <<"miOffset="<<miOffset<<std::endl;
	//while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
	while (  contLin<maxLine ){
		    //std::cout <<"contLin="<<contLin<<std::endl;

	//while ( infile >> rx >> ry >> rz  ){
			//std::cout << "You said.... contLin"<< contLin << timestamp << " " << rx << " " << ry << " " << rz << " " << q1 << " " << q2 << " " << q3 << " " << q4 << "\n";
			//VectorXd myVector (rx,ry,rz);
			//readingA.row(contLin)= myVector;
		    //create sequence 1, two variables , T (time), X
			//readingA.row(contLin)<< timestamp,rx,ry,rz;
			//readingA.row(contLin)<< contLin,rx,ry,rz;
		    readingA.row(contLin)<< contLin,sin(contLin),ry,rz;
		    vTiempoA.row(contLin) << contLin;
		    vSerieA.row(contLin) << sin(contLin);
			//vTiempoA.row(contLin) << timestamp;
			//create sequence 2, two varialbes,  T+offset , X
			//readingB.row(contLin)<< timestamp+miOffset,rx,ry,rz;
		    newContLin= contLin+miOffset;
		    std::cout <<"newContLin="<<newContLin<<std::endl;
			readingB.row(contLin)<< newContLin,sin(contLin),ry,rz;
			vTiempoB.row(contLin) << newContLin;
			vSerieB.row(contLin) << sin(contLin);

			outA << contLin <<" "<< sin(contLin) <<" "<< ry <<" "<< rz <<" "<< q1 <<" "<< q2 <<" "<<q3 <<" "<< q4<<std::endl;
			outB << newContLin <<" "<< sin(contLin) <<" "<< ry <<" "<< rz <<" "<< q1 <<" "<< q2 <<" "<<q3 <<" "<< q4<<std::endl;
			contLin ++;

	}

	outA.close();
	outB.close();
	contLin=maxLine;
	std::cout <<"contLin="<<contLin<<std::endl;
	//Calcular autoCorrelacion
	MatrixXd A (contLin,3);
	MatrixXd B (contLin,3);
	A = readingA.block(0,0,contLin,3);
	B = readingB.block(0,0,contLin,3);
	mediaTiempoA = vTiempoA.mean();
	std::cout <<"	mediaTiempoA="<<	mediaTiempoA<<std::endl;

	mediaTiempoB = vTiempoB.mean();
	std::cout <<"	mediaTiempoB="<<	mediaTiempoB<<std::endl;
	mediaSerieA = vSerieA.mean();
	std::cout <<"	mediaSerieA="<<	mediaSerieA<<std::endl;

	mediaSerieB = vSerieB.mean();
	std::cout <<"	mediaSerieB="<<	mediaSerieB<<std::endl;

	VectorXd centeredTA,centeredTB,totalCenteredT;
	//for (int i=0; i < contLin ; i++){
		//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

	//}
	centeredTA = vTiempoA.array() - mediaTiempoA;
	std::cout <<"	centeredTA="<<	centeredTA<<std::endl;
	centeredTB = vTiempoB.array() - mediaTiempoB;
	std::cout <<"	centeredTB="<<	centeredTB<<std::endl;

	//Serie
	VectorXd centeredSA,centeredSB,totalCenteredS;
		//for (int i=0; i < contLin ; i++){
			//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

		//}
	centeredSA = vSerieA.array() - mediaSerieA;
	std::cout <<"	centeredSA="<<	centeredSA<<std::endl;
	centeredSB = vSerieB.array() - mediaSerieB;
	std::cout <<"	centeredSB="<<	centeredSB<<std::endl;

	double denom=0,sx=0,sy=0;
	for ( int i=0; i < contLin; i++){
			//sx += centeredTA(i)*centeredTA(i);
			//sy += centeredTB(i)*centeredTB(i);
			sx += centeredSA(i)*centeredSA(i);
			sy += centeredSB(i)*centeredSB(i);
	}
	//denom = sqrt(sx*sy);
	denom = sqrt(sx*sy);

	/* Calculate the correlation series */
       double sxy=0, rMax=0;
       int j = 0,delayMax=0;

	  for (int delay=-intervalo;delay<=intervalo;delay++) {
		  sxy = 0;
		  for (int i=0;i<contLin;i++) {
			 j = i + delay;
			 if (j < 0 || j >= maxLine)
				continue;
			 else
				//sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
				 //sxy += centeredTA[i] * centeredTB[j];
				 sxy += centeredSA[i] * centeredSB[j];

		  }



		  double r = sxy / denom;
          if ((r < 1 )&& (fabs(r) > rMax)){
        	  rMax=r;
        	  delayMax=delay;
          }
	      /* r is the correlation coefficient at "delay" */
		  std::cout <<"r ="<<fabs(r)<<std::endl;
         std::cout <<"delay ="<<delay<<std::endl;
		  //std::cout <<"i="<<i<<std::endl;
	   }
	  std::cout <<"rMax="<<rMax<<std::endl;
	  std::cout <<"delayMax="<<delayMax<<std::endl;


}
void AjusteTiempo::calculateCrossCorrelation(int maxLine, int intervalo,double offset, MatrixXd& A, MatrixXd& B)

{
	// offset: define la separación de la segunda serie que sera una serie artificial. La segunda serie S2 = S1 + offset
	// intervalo: define la amplitud de la ventana desde la que buscaremos la correlación
	// maxLine : numero máximo de líneas


    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    outRegresion << std::setprecision(6) << std::fixed;


	//int maxLine = 30;
	//maxLine=maxLine;
	MatrixXd readingA (maxLine,4);//size is big enough to store an unknown number of data rows from file
	MatrixXd readingB (int(maxLine+offset),4);
	VectorXd vTiempoA (maxLine), vSerieA (maxLine) ;
	VectorXd vTiempoB (int(maxLine+offset)), vSerieB (int(maxLine+offset)) ;
	//read a file
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	double mediaTiempoA, mediaTiempoB, mediaSerieA, mediaSerieB;
	std::cout <<"antes de leer el archivo"<<std::endl;
	//int miOffset=5;
    double miOffset = offset;
    double newContLin = 0;
    std::cout <<"miOffset="<<miOffset<<std::endl;
    /* initialize random seed: */
    srand (time(NULL));
	//while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
    double aleatorio =0, valueToB=0;


	contLin=maxLine;
	std::cout <<"contLin="<<contLin<<std::endl;
	//Calcular autoCorrelacion
	//MatrixXd A (contLin,3);
	//MatrixXd B (contLin,3);
	//A = readingA.block(0,0,contLin,3);
	//B = readingB.block(0,0,contLin,3);
    vSerieA = A.col(0).array();
    vSerieB = B.col(0).array();
	mediaSerieA = vSerieA.mean();
	std::cout <<"	mediaSerieA="<<	mediaSerieA<<std::endl;

	mediaSerieB = vSerieB.mean();
	std::cout <<"	mediaSerieB="<<	mediaSerieB<<std::endl;


	//Serie
	VectorXd centeredSA,centeredSB,totalCenteredS;
		//for (int i=0; i < contLin ; i++){
			//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

		//}
	centeredSA = vSerieA.array() - mediaSerieA;
	std::cout <<"	centeredSA="<<	centeredSA<<std::endl;
	centeredSB = vSerieB.array() - mediaSerieB;
	std::cout <<"	centeredSB="<<	centeredSB<<std::endl;

	double denom=0,sx=0,sy=0;
	for ( int i=0; i < contLin; i++){
			//sx += centeredTA(i)*centeredTA(i);
			//sy += centeredTB(i)*centeredTB(i);
			sx += centeredSA(i)*centeredSA(i);
			sy += centeredSB(i)*centeredSB(i);
	}

	denom = sqrt(sx*sy);

	/* Calculate the correlation series */
       double sxy=0, rMax=0;
       int j = 0,delayMax=0;

	  for (int delay=-intervalo;delay<intervalo;delay++) {
		  sxy = 0;
		  sx=0;
		  sy=0;
		  for (int i=0;i<contLin;i++) {
			 j = i + delay;
			 if (j < 0 || j >= maxLine)
				continue;
			 else
				//sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
				 //sxy += centeredTA[i] * centeredTB[j];
				 //sxy += centeredSA[i] * centeredSB[j];
				 sxy += (vSerieA[i] - mediaSerieA) * (vSerieB[j] - mediaSerieB);
                 //sx += centeredSA(i)*centeredSA(i);
			     //sy += centeredSB(j)*centeredSB(j);
		  }



		  double r = sxy / denom;
		  //double r = sxy / sqrt(sx*sy);
          if ((fabs(r) < 1 )&& (fabs(r) > rMax)){
        	  rMax=r;
        	  delayMax=delay;
          }
	      /* r is the correlation coefficient at "delay" */
		  std::cout <<"r ="<<fabs(r)<<std::endl;
         std::cout <<"delay ="<<delay<<std::endl;
         outRegresion<< r << " " << delay << std::endl;
		  //std::cout <<"i="<<i<<std::endl;
	   }
	  std::cout <<"rMax="<<rMax<<std::endl;
	  std::cout <<"delayMax="<<delayMax<<std::endl;
	  outRegresion.close();


}
void AjusteTiempo::calcularAutocorrelacion2(int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2)

{
	// offset: define la separación de la segunda serie que sera una serie artificial. La segunda serie S2 = S1 + offset
	// intervalo: define la amplitud de la ventana desde la que buscaremos la correlación
	// maxLine : numero máximo de líneas


    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    outRegresion << std::setprecision(6) << std::fixed;

	std::cout <<"antes de declarar la variable"<<std::endl;
	//int maxLine = 30;
	maxLine=maxLine;
	MatrixXd readingA (maxLine,4);//size is big enough to store an unknown number of data rows from file
	MatrixXd readingB (int(maxLine+offset),4);
	VectorXd vTiempoA (maxLine), vSerieA (maxLine) ;
	VectorXd vTiempoB (int(maxLine+offset)), vSerieB (int(maxLine+offset)) ;
	//read a file
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	double mediaTiempoA, mediaTiempoB, mediaSerieA, mediaSerieB;
	std::cout <<"antes de leer el archivo"<<std::endl;
	//int miOffset=5;
    double miOffset = offset;
    double newContLin = 0;
    std::cout <<"miOffset="<<miOffset<<std::endl;
    /* initialize random seed: */
    srand (time(NULL));
	//while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
    double aleatorio =0, valueToB=0;
	while (  contLin<maxLine ){
		   std::cout <<"contLin="<<contLin<<std::endl;

	//while ( infile >> rx >> ry >> rz  ){
			//std::cout << "You said.... contLin"<< contLin << timestamp << " " << rx << " " << ry << " " << rz << " " << q1 << " " << q2 << " " << q3 << " " << q4 << "\n";
			//VectorXd myVector (rx,ry,rz);
			//readingA.row(contLin)= myVector;
		    //create sequence 1, two variables , T (time), X
			//readingA.row(contLin)<< timestamp,rx,ry,rz;
			//readingA.row(contLin)<< contLin,rx,ry,rz;
		    //readingA.row(contLin)<< contLin,sin(contLin),ry,rz;
		    //vTiempoA.row(contLin) << contLin;
		    vSerieA.row(contLin) << sin(contLin);
			//vTiempoA.row(contLin) << timestamp;
			//create sequence 2, two varialbes,  T+offset , X
			//readingB.row(contLin)<< timestamp+miOffset,rx,ry,rz;
		    //double num =  rand() % 2 + 1;
		    //double num = (float) rand()/RAND_MAX +0.5;
		    //std::cout <<"num="<<num<<std::endl;

		    //std::cout <<"newContLin="<<newContLin<<std::endl;
			//readingB.row(contLin)<< newContLin,sin(contLin),ry,rz;
			//vTiempoB.row(contLin) << newContLin;
			//vSerieB.row(contLin) << sin(contLin)+num;
			//vSerieB.row(contLin) << sin(contLin)+sqrt(contLin/50);


			outA << contLin <<" "<< sin(contLin) <<std::endl;

			newContLin= contLin + miOffset;
			//if (contLin < 65) {
			if (newContLin < 86) {
				 //aleatorio = cos((double) rand()/RAND_MAX +0.5);
				//aleatorio = cos((double) rand()/RAND_MAX +1);
                //valueToB = newContLin/(aleatorio + 1);
				aleatorio = ((double) rand()/RAND_MAX +0.5);
                //valueToB = (newContLin/9)+ 1/(aleatorio);
				valueToB = (newContLin*newContLin)/37 - 2*(newContLin+aleatorio);
			} else {
				aleatorio = 0;
				valueToB = sin(contLin);
			}

			//valueToB=sin(contLin);
		    vSerieB.row(newContLin)<<valueToB;
			outB << newContLin <<" "<< valueToB <<std::endl;
			contLin ++;

	}
	std::cout <<"fin de bucle"<<contLin<<std::endl;
	outA.close();
	outB.close();
	contLin=maxLine;
	std::cout <<"contLin="<<contLin<<std::endl;
	//Calcular autoCorrelacion
	MatrixXd A (contLin,3);
	MatrixXd B (contLin,3);
	A = readingA.block(0,0,contLin,3);
	B = readingB.block(0,0,contLin,3);

	mediaSerieA = vSerieA.mean();
	std::cout <<"	mediaSerieA="<<	mediaSerieA<<std::endl;

	mediaSerieB = vSerieB.mean();
	std::cout <<"	mediaSerieB="<<	mediaSerieB<<std::endl;


	//Serie
	VectorXd centeredSA,centeredSB,totalCenteredS;
		//for (int i=0; i < contLin ; i++){
			//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

		//}
	centeredSA = vSerieA.array() - mediaSerieA;
	std::cout <<"	centeredSA="<<	centeredSA<<std::endl;
	centeredSB = vSerieB.array() - mediaSerieB;
	std::cout <<"	centeredSB="<<	centeredSB<<std::endl;

	double denom=0,sx=0,sy=0;
	for ( int i=0; i < contLin; i++){
			//sx += centeredTA(i)*centeredTA(i);
			//sy += centeredTB(i)*centeredTB(i);
			sx += centeredSA(i)*centeredSA(i);
			sy += centeredSB(i)*centeredSB(i);
	}

	denom = sqrt(sx*sy);

	/* Calculate the correlation series */
       double sxy=0, rMax=0;
       int j = 0,delayMax=0;

	  for (int delay=-intervalo;delay<intervalo;delay++) {
		  sxy = 0;
		  sx=0;
		  sy=0;
		  for (int i=0;i<contLin;i++) {
			 j = i + delay;
			 if (j < 0 || j >= maxLine)
				continue;
			 else
				//sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
				 //sxy += centeredTA[i] * centeredTB[j];
				 //sxy += centeredSA[i] * centeredSB[j];
				 sxy += (vSerieA[i] - mediaSerieA) * (vSerieB[j] - mediaSerieB);
                 //sx += centeredSA(i)*centeredSA(i);
			     //sy += centeredSB(j)*centeredSB(j);
		  }



		  double r = sxy / denom;
		  //double r = sxy / sqrt(sx*sy);
          if ((fabs(r) < 1 )&& (fabs(r) > rMax)){
        	  rMax=r;
        	  delayMax=delay;
          }
	      /* r is the correlation coefficient at "delay" */
		  std::cout <<"r ="<<fabs(r)<<std::endl;
         std::cout <<"delay ="<<delay<<std::endl;
         outRegresion<< r << " " << delay << std::endl;
		  //std::cout <<"i="<<i<<std::endl;
	   }
	  std::cout <<"rMax="<<rMax<<std::endl;
	  std::cout <<"delayMax="<<delayMax<<std::endl;
	  outRegresion.close();


}

void AjusteTiempo::calcularAutocorrelacion3(char coordinate, int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2)

{
	// offset: define la separación de la segunda serie que sera una serie artificial. La segunda serie S2 = S1 + offset
	// intervalo: define la amplitud de la ventana desde la que buscaremos la correlación
	// maxLine : numero máximo de líneas

	std::cout << std::setprecision(6) << std::fixed;
	std::ifstream inputFile( "/home/tfm3/workspace/AjusteTiempo/original_data.txt" );
	//inputFile >> std::setprecision(6) >> std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    outRegresion << std::setprecision(6) << std::fixed;

	std::cout <<"antes de declarar la variable"<<std::endl;
	//int maxLine = 30;
	maxLine=maxLine;
	MatrixXd readingA (maxLine,4);//size is big enough to store an unknown number of data rows from file
	MatrixXd readingB (int(maxLine+offset),4);
	VectorXd vTiempoA (maxLine), vSerieA (maxLine) ;
	VectorXd vTiempoB (int(maxLine+offset)), vSerieB (int(maxLine+offset)) ;
	//read a file
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	double mediaTiempoA, mediaTiempoB, mediaSerieA, mediaSerieB;
	std::cout <<"antes de leer el archivo"<<std::endl;
	//int miOffset=5;
    double miOffset = offset;
    double newContLin = 0;
    std::cout <<"miOffset="<<miOffset<<std::endl;
    /* initialize random seed: */
    srand (time(NULL));
	//while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
    double aleatorio =0, valueToB=0, valueToA=0;
	//while (  contLin<maxLine ){
	//	   std::cout <<"contLin="<<contLin<<std::endl;

	while ( inputFile  >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4 && contLin < maxLine ){
			//std::cout << "You said.... contLin"<< contLin << timestamp << " " << rx << " " << ry << " " << rz << " " << q1 << " " << q2 << " " << q3 << " " << q4 << "\n";
			//VectorXd myVector (rx,ry,rz);
			//readingA.row(contLin)= myVector;
		    //create sequence 1, two variables , T (time), X
			//readingA.row(contLin)<< timestamp,rx,ry,rz;
			//readingA.row(contLin)<< contLin,rx,ry,rz;
		    //readingA.row(contLin)<< contLin,sin(contLin),ry,rz;
		    //vTiempoA.row(contLin) << contLin;
		    if (coordinate == 'x')//read Coordinate X
		    	valueToA = rx;

		    else if (coordinate == 'y')

		    	valueToA = ry;

			 else if (coordinate == 'z')
				valueToA = rz;


		    vSerieA.row(contLin)<<valueToA;
			//vTiempoA.row(contLin) << timestamp;
			//create sequence 2, two varialbes,  T+offset , X
			//readingB.row(contLin)<< timestamp+miOffset,rx,ry,rz;
		    //double num =  rand() % 2 + 1;
		    //double num = (float) rand()/RAND_MAX +0.5;
		    //std::cout <<"num="<<num<<std::endl;

		    //std::cout <<"newContLin="<<newContLin<<std::endl;
			//readingB.row(contLin)<< newContLin,sin(contLin),ry,rz;
			//vTiempoB.row(contLin) << newContLin;
			//vSerieB.row(contLin) << sin(contLin)+num;
			//vSerieB.row(contLin) << sin(contLin)+sqrt(contLin/50);


			outA << contLin <<" "<< valueToA <<std::endl;

			newContLin= contLin + miOffset;
			if (contLin < 0) {
			//if (newContLin < 200) {
				 //aleatorio = cos((double) rand()/RAND_MAX +0.5);
				//aleatorio = cos((double) rand()/RAND_MAX +1);
                //valueToB = newContLin/(aleatorio + 1);
				aleatorio = ((double) rand()/RAND_MAX +0.5);
                //valueToB = (newContLin/9)+ 1/(aleatorio);
				valueToB = (newContLin*newContLin)/150 - 2*(newContLin+aleatorio);
				//valueToB = 7*sin(newContLin);
			} else {
				aleatorio = 0;
				valueToB = valueToA;
			}

			//valueToB=sin(contLin);
		    vSerieB.row(newContLin)<<valueToB;
			outB << newContLin <<" "<< valueToB <<std::endl;
			contLin ++;

	}
	std::cout <<"fin de bucle"<<contLin<<std::endl;
	inputFile.close();
	outA.close();
	outB.close();
	contLin=maxLine;
	std::cout <<"contLin="<<contLin<<std::endl;
	//Calcular autoCorrelacion
	MatrixXd A (contLin,3);
	MatrixXd B (contLin,3);
	A = readingA.block(0,0,contLin,3);
	B = readingB.block(0,0,contLin,3);

	mediaSerieA = vSerieA.mean();
	std::cout <<"	mediaSerieA="<<	mediaSerieA<<std::endl;

	mediaSerieB = vSerieB.mean();
	std::cout <<"	mediaSerieB="<<	mediaSerieB<<std::endl;


	//Serie
	VectorXd centeredSA,centeredSB,totalCenteredS;
		//for (int i=0; i < contLin ; i++){
			//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

		//}
	centeredSA = vSerieA.array() - mediaSerieA;
	//std::cout <<"	centeredSA="<<	centeredSA<<std::endl;
	centeredSB = vSerieB.array() - mediaSerieB;
	//std::cout <<"	centeredSB="<<	centeredSB<<std::endl;

	double denom=0,sx=0,sy=0;
	for ( int i=0; i < contLin; i++){
			//sx += centeredTA(i)*centeredTA(i);
			//sy += centeredTB(i)*centeredTB(i);
			sx += centeredSA(i)*centeredSA(i);
			sy += centeredSB(i)*centeredSB(i);
	}

	denom = sqrt(sx*sy);

	/* Calculate the correlation series */
       double sxy=0, rMax=0;
       int j = 0,delayMax=0;

	  for (int delay=-intervalo;delay<intervalo;delay++) {
		  sxy = 0;
		  sx=0;
		  sy=0;
		  for (int i=0;i<contLin;i++) {
			 j = i + delay;
			 if (j < 0 || j >= maxLine)
				continue;
			 else
				//sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
				 //sxy += centeredTA[i] * centeredTB[j];
				 //sxy += centeredSA[i] * centeredSB[j];
				 sxy += (vSerieA[i] - mediaSerieA) * (vSerieB[j] - mediaSerieB);
                 //sx += centeredSA(i)*centeredSA(i);
			     //sy += centeredSB(j)*centeredSB(j);
		  }



		  double r = sxy / denom;
		  //double r = sxy / sqrt(sx*sy);
          if ((fabs(r) <= 1 )&& (fabs(r) > rMax)){
        	  rMax=r;
        	  delayMax=delay;
          }
	      /* r is the correlation coefficient at "delay" */
		  //std::cout <<"r ="<<fabs(r)<<std::endl;
         //std::cout <<"delay ="<<delay<<std::endl;
         outRegresion<< r << " " << delay << std::endl;
		  //std::cout <<"i="<<i<<std::endl;
	   }
	  std::cout <<"rMax="<<rMax<<std::endl;
	  std::cout <<"delayMax="<<delayMax<<std::endl;
	  outRegresion.close();


}

//double AjusteTiempo::calculateOffsetXYZ(int maxLine, int interval, double offset, MatrixXd& A1,MatrixXd& B2){
double AjusteTiempo::calculateOffsetXYZ(int maxLine, int interval, double offset, double& offsetEstimated, MatrixXd A1,MatrixXd B2){
	//Calculate Correlation for 2 matrix, each has 3 columns and maxLine rows. Use with 3d datasets
	//offset: defines gap between second matrix and first matrix
	//maxLine: number of lines
	//interval: defines variation to calculate offset. If interval = 100, then calculate offset from -100 to 100
	std::cout<<" CALCULATE OFFSET XYZ......................................."<<std::endl;
	std::cout << std::setprecision(6) << std::fixed;
	std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
	outA << std::setprecision(6) << std::fixed;

	std::cout << std::setprecision(6) << std::fixed;
	std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
	outB << std::setprecision(6) << std::fixed;

	std::cout << std::setprecision(6) << std::fixed;
	std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
	outRegresion << std::setprecision(6) << std::fixed;

	int contLin=maxLine;

    std::cout <<"contLin="<<A1.rows()<<std::endl;

    std::cout <<"contLin2"<<B2.rows()<<std::endl;

	MatrixXd readingB= MatrixXd::Zero((maxLine+offset),B2.cols());
    //for (int h=0; h<( maxLine-10);h++){
    for (int h=0; h<B2.rows() ; h++){

        readingB.row(int(h+offset)) = B2.row(h);
        std::cout <<"h==="<<h<<std::endl;
	}
    std::cout <<"fin bucle h"<<std::endl;

	//mediaSerieA = A1.col(1).mean();
	double mediaSerieA0 = A1.col(0).mean();
	double mediaSerieA1 = A1.col(1).mean();
	double mediaSerieA2 = A1.col(2).mean();

    std::cout <<"	mediaSerieA="<<	round(mediaSerieA0)<<std::endl;

	//mediaSerieB = B2.col(1).mean();
	double mediaSerieB0 = B2.col(0).mean();
	double mediaSerieB1 = B2.col(1).mean();
	double mediaSerieB2 = B2.col(2).mean();

    std::cout <<"	mediaSerieB="<<	round(mediaSerieB0)<<std::endl;
//Serie
	VectorXd centeredSA,centeredSB,totalCenteredS;
		//for (int i=0; i < contLin ; i++){
			//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

		//}
	VectorXd centeredSA0 = A1.col(0).array() - mediaSerieA0;
	VectorXd centeredSA1 = A1.col(1).array() - mediaSerieA1;
	VectorXd centeredSA2 = A1.col(2).array() - mediaSerieA2;
	//centeredSA = A1.col(1).array() - mediaSerieA;
	//std::cout <<"	centeredSA="<<	centeredSA<<std::endl;
	//centeredSB = B2.col(1).array() - mediaSerieB;
	VectorXd centeredSB0 = B2.col(0).array() - mediaSerieB0;
	VectorXd centeredSB1 = B2.col(1).array() - mediaSerieB1;
	VectorXd centeredSB2 = B2.col(2).array() - mediaSerieB2;
	//std::cout <<"	centeredSB="<<	centeredSB<<std::endl;

	double denom=0,sx=0,sy=0,sx0=0,sx1=0,sx2=0,sy0=0,sy1=0,sy2=0;
	for ( int i=0; i < contLin; i++){

			//sx += centeredTA(i)*centeredTA(i);
			//sy += centeredTB(i)*centeredTB(i);
			sx0 += centeredSA0(i)*centeredSA0(i);
			sx1 += centeredSA1(i)*centeredSA1(i);
			sx2 += centeredSA2(i)*centeredSA2(i);

 			sy0 += centeredSB0(i)*centeredSB0(i);
 			sy1 += centeredSB1(i)*centeredSB1(i);
 			sy2 += centeredSB2(i)*centeredSB2(i);



	}

	denom = sqrt((sx0+sx1+sx2)*(sy0+sy1+sy2));

	// Calculate the correlation series
	double sxy0=0,sxy1=0,sxy2=0, rMax=0;
	int j = 0,delayMax=0;

	std::cout <<"contLin3="<<contLin<<std::endl;
	std::cout <<"maxLine="<<maxLine<<std::endl;

	for (int delay=-interval;delay<interval;delay++) {
		  sxy0 = 0;
		  sxy1 = 0;
		  sxy2 = 0;
		  sx=0;
		  sy=0;
		  //std::cout <<"delay ="<<delay<<std::endl;
		  //for (int i=0;i<contLin;i++) {
		  for (int i=0;i<maxLine;i++) {
			 //std::cout <<"i ="<<i<<" j="<<j<<std::endl;
			 j = i + delay;
			 if (j < 0 || j >= maxLine)
				continue;
             else{
				//sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
				 //sxy += centeredTA[i] * centeredTB[j];
				 //sxy += centeredSA[i] * centeredSB[j];
				 //sxy += ((A1.row(i))(1) - mediaSerieA) * ((readingB.row(j))(1) - mediaSerieB);
				 sxy0 += ((A1.row(i))(0) - mediaSerieA0) * ((readingB.row(j))(0) - mediaSerieB0);
			     sxy1 += ((A1.row(i))(1) - mediaSerieA1) * ((readingB.row(j))(1) - mediaSerieB1);
			     sxy2 += ((A1.row(i))(2) - mediaSerieA2) * ((readingB.row(j))(2) - mediaSerieB2);
				 //sx += centeredSA(i)*centeredSA(i);
				 //sy += centeredSB(j)*centeredSB(j);
             }
		  }



		  double r = (sxy0+sxy1+sxy2) / denom;
		  //double r = sxy / sqrt(sx*sy);
		  if ((fabs(r) <= 1 )&& (fabs(r) > rMax)){
			  rMax=r;
			  delayMax=delay;
		  }
		  // r is the correlation coefficient at "delay"
		 //std::cout <<"r ="<<fabs(r)<<std::endl;
		 //std::cout <<"delay ="<<delay<<std::endl;
		 outRegresion<< r << " " << delay << std::endl;
		  //std::cout <<"i="<<i<<std::endl;
	   }
	  std::cout <<"rMax="<<rMax<<std::endl;
	  std::cout <<"delayMax (offset)="<<delayMax<<std::endl;
	  outRegresion.close();
      offsetEstimated = delayMax;
	  return rMax;

}

double AjusteTiempo::calculateOffsetTXYZ(int maxLine, int interval, MatrixXd A1,MatrixXd B2){
    //Calculate Correlation for 2 matrix, each has 3 columns and maxLine rows. Use with 3d datasets
    //offset: defines gap between second matrix and first matrix
    //maxLine: number of lines
    //interval: defines variation to calculate offset. If interval = 100, then calculate offset from -100 to 100
    std::cout<<" CALCULATE OFFSET TXYZ......................................."<<std::endl;
    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    //std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    //outRegresion << std::setprecision(6) << std::fixed;

    int contLin=maxLine;

    std::cout <<"contLin="<<A1.rows()<<std::endl;

    std::cout <<"contLin2"<<B2.rows()<<std::endl;

    MatrixXd readingB= MatrixXd::Zero((B2.rows()),B2.cols());
    //for (int h=0; h<( maxLine-10);h++){
    //for (int h=0; h<B2.rows() ; h++){

        //readingB.row(int(h+offset)) = B2.row(h);
        //readingB.row(h) = B2.row(h);
        //std::cout <<"h==="<<h<<std::endl;
    //}
    readingB=B2.block(0,0,B2.rows(),B2.cols());
    std::cout <<"fin bucle h"<<std::endl;

    //mediaSerieA = A1.col(1).mean();
    double mediaSerieA0 = A1.col(1).mean();
    double mediaSerieA1 = A1.col(2).mean();
    double mediaSerieA2 = A1.col(3).mean();

    //std::cout <<"	mediaSerieA="<<	round(mediaSerieA0)<<std::endl;

    //mediaSerieB = B2.col(1).mean();
    double mediaSerieB0 = B2.col(1).mean();
    double mediaSerieB1 = B2.col(2).mean();
    double mediaSerieB2 = B2.col(3).mean();

    //std::cout <<"	mediaSerieB="<<	round(mediaSerieB0)<<std::endl;
//Serie
    VectorXd centeredSA,centeredSB,totalCenteredS;
        //for (int i=0; i < contLin ; i++){
            //centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

        //}
    VectorXd centeredSA0 = A1.col(1).array() - mediaSerieA0;
    VectorXd centeredSA1 = A1.col(2).array() - mediaSerieA1;
    VectorXd centeredSA2 = A1.col(3).array() - mediaSerieA2;
    //centeredSA = A1.col(1).array() - mediaSerieA;
    //std::cout <<"	centeredSA0="<<	centeredSA0<<std::endl;
    //centeredSB = B2.col(1).array() - mediaSerieB;
    VectorXd centeredSB0 = B2.col(1).array() - mediaSerieB0;
    VectorXd centeredSB1 = B2.col(2).array() - mediaSerieB1;
    VectorXd centeredSB2 = B2.col(3).array() - mediaSerieB2;
    //std::cout <<"	centeredSB0="<<	centeredSB0<<std::endl;

    double denom=0,sx=0,sy=0,sx0=0,sx1=0,sx2=0,sy0=0,sy1=0,sy2=0;
    for ( int i=0; i < A1.rows(); i++){

            //sx += centeredTA(i)*centeredTA(i);
            //sy += centeredTB(i)*centeredTB(i);
            sx0 += centeredSA0(i)*centeredSA0(i);
            sx1 += centeredSA1(i)*centeredSA1(i);
            sx2 += centeredSA2(i)*centeredSA2(i);
    }
    for ( int i=0; i < B2.rows(); i++){
            sy0 += centeredSB0(i)*centeredSB0(i);
            sy1 += centeredSB1(i)*centeredSB1(i);
            sy2 += centeredSB2(i)*centeredSB2(i);



    }

    denom = sqrt((sx0+sx1+sx2)*(sy0+sy1+sy2));

    // Calculate the correlation series
    double sxy0=0,sxy1=0,sxy2=0, rMax=0;
    double delayMax=0;
    int i=0,j = 0;

    std::cout <<"contLin3="<<contLin<<std::endl;
    std::cout <<"maxLine="<<maxLine<<std::endl;
    int maxRowsA = A1.rows();
    int maxRowsB = B2.rows();


    while (i < maxRowsA && j < maxRowsB){


             //j = i + offset;
             j=i;
             if (j < 0 || j >= maxRowsB)
                continue;
             else{
                //sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
                 //sxy += centeredTA[i] * centeredTB[j];
                 //sxy += centeredSA[i] * centeredSB[j];
                 sxy0 += ((A1.row(i))(1) - mediaSerieA0) * ((B2.row(j))(1) - mediaSerieB0);
                 sxy1 += ((A1.row(i))(2) - mediaSerieA1) * ((B2.row(j))(2) - mediaSerieB1);
                 sxy2 += ((A1.row(i))(3) - mediaSerieA2) * ((B2.row(j))(3) - mediaSerieB2);
                 //sx += centeredSA(i)*centeredSA(i);
                 //sy += centeredSB(j)*centeredSB(j);
              }
             i++;
          }



          double r = (sxy0+sxy1+sxy2) / denom;
          //double r = sxy / sqrt(sx*sy);

          // r is the correlation coefficient at "delay"
          std::cout <<"r ="<<fabs(r)<<std::endl;
          //std::cout <<"delay ="<<offset<<std::endl;
          return r;


}

double AjusteTiempo::calculateOffsetTXYZ2(int maxLine, int interval, MatrixXd A1,MatrixXd B2){
    //Calculate Correlation for 2 matrix, each has 3 columns and maxLine rows. Use with 3d datasets
    //offset: defines gap between second matrix and first matrix
    //maxLine: number of lines
    //interval: defines variation to calculate offset. If interval = 100, then calculate offset from -100 to 100
    std::cout<<" CALCULATE OFFSET TXYZ2......................................."<<std::endl;
    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    //std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    //outRegresion << std::setprecision(6) << std::fixed;

    int contLin=maxLine;

    std::cout <<"contLin="<<A1.rows()<<std::endl;

    std::cout <<"contLin2"<<B2.rows()<<std::endl;

    MatrixXd readingB= MatrixXd::Zero((B2.rows()),B2.cols());
    //for (int h=0; h<( maxLine-10);h++){
    //for (int h=0; h<B2.rows() ; h++){

        //readingB.row(int(h+offset)) = B2.row(h);
        //readingB.row(h) = B2.row(h);
        //std::cout <<"h==="<<h<<std::endl;
    //}
    readingB=B2.block(0,0,B2.rows(),B2.cols());
    std::cout <<"fin bucle h"<<std::endl;

    //mediaSerieA = A1.col(1).mean();
    double mediaSerieA0 = A1.col(1).mean();
    double mediaSerieA1 = A1.col(2).mean();
    double mediaSerieA2 = A1.col(3).mean();

    //std::cout <<"	mediaSerieA="<<	round(mediaSerieA0)<<std::endl;

    //mediaSerieB = B2.col(1).mean();
    double mediaSerieB0 = B2.col(1).mean();
    double mediaSerieB1 = B2.col(2).mean();
    double mediaSerieB2 = B2.col(3).mean();

    //std::cout <<"	mediaSerieB="<<	round(mediaSerieB0)<<std::endl;
//Serie
    VectorXd centeredSA,centeredSB,totalCenteredS;
        //for (int i=0; i < contLin ; i++){
            //centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

        //}
    VectorXd centeredSA0 = A1.col(1).array() - mediaSerieA0;
    VectorXd centeredSA1 = A1.col(2).array() - mediaSerieA1;
    VectorXd centeredSA2 = A1.col(3).array() - mediaSerieA2;
    //centeredSA = A1.col(1).array() - mediaSerieA;
    //std::cout <<"	centeredSA0="<<	centeredSA0<<std::endl;
    //centeredSB = B2.col(1).array() - mediaSerieB;
    VectorXd centeredSB0 = B2.col(1).array() - mediaSerieB0;
    VectorXd centeredSB1 = B2.col(2).array() - mediaSerieB1;
    VectorXd centeredSB2 = B2.col(3).array() - mediaSerieB2;
    //std::cout <<"	centeredSB0="<<	centeredSB0<<std::endl;

    double denom=0,sx=0,sy=0,sx0=0,sx1=0,sx2=0,sy0=0,sy1=0,sy2=0;
    for ( int i=0; i < A1.rows(); i++){

            //sx += centeredTA(i)*centeredTA(i);
            //sy += centeredTB(i)*centeredTB(i);
            sx0 += centeredSA0(i)*centeredSA0(i);
            sx1 += centeredSA1(i)*centeredSA1(i);
            sx2 += centeredSA2(i)*centeredSA2(i);
    }
    for ( int i=0; i < B2.rows(); i++){
            sy0 += centeredSB0(i)*centeredSB0(i);
            sy1 += centeredSB1(i)*centeredSB1(i);
            sy2 += centeredSB2(i)*centeredSB2(i);



    }

    denom = sqrt((sx0+sx1+sx2)*(sy0+sy1+sy2));

    // Calculate the correlation series
    double sxy0=0,sxy1=0,sxy2=0, rMax=0;
    double delayMax=0;
    int i=0,j = 0;

    std::cout <<"contLin3="<<contLin<<std::endl;
    std::cout <<"maxLine="<<maxLine<<std::endl;
    int maxRowsA = A1.rows();
    int maxRowsB = B2.rows();
    for (int delay=-interval;delay<interval;delay++) {
          sxy0 = 0;
          sxy1 = 0;
          sxy2 = 0;
          sx=0;
          sy=0;
          for (int i=0;i<maxLine;i++) {
             //std::cout <<"i ="<<i<<" j="<<j<<std::endl;
             j = i + delay;
             if (j < 0 || j >= maxLine)
                continue;
             else {
                 //sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
                 //sxy += centeredTA[i] * centeredTB[j];
                 //sxy += centeredSA[i] * centeredSB[j];
                 sxy0 += ((A1.row(i))(1) - mediaSerieA0) * ((B2.row(j))(1) - mediaSerieB0);
                 sxy1 += ((A1.row(i))(2) - mediaSerieA1) * ((B2.row(j))(2) - mediaSerieB1);
                 sxy2 += ((A1.row(i))(3) - mediaSerieA2) * ((B2.row(j))(3) - mediaSerieB2);
                 //sx += centeredSA(i)*centeredSA(i);
                 //sy += centeredSB(j)*centeredSB(j);
             }
             //i++;
          }
             double r = (sxy0+sxy1+sxy2) / denom;
             //double r = sxy / sqrt(sx*sy);

             // r is the correlation coefficient at "delay"
             std::cout <<"r ="<<fabs(r)<<std::endl;
             //std::cout <<"delay ="<<offset<<std::endl;
             if ((fabs(r) <= 1 )&& (fabs(r) > rMax)){
                 rMax=r;
                 delayMax=delay;
             }
    }




          return rMax;


}

double AjusteTiempo::calculateOffsetTXYZ3(int maxLine, int interval, MatrixXd A1,MatrixXd B2){
    //Calculate Correlation for 2 matrix, each has 3 columns and maxLine rows. Use with 3d datasets
    //offset: defines gap between second matrix and first matrix
    //maxLine: number of lines
    //interval: defines variation to calculate offset. If interval = 100, then calculate offset from -100 to 100
    std::cout<<" CALCULATE OFFSET TXYZ3......................................."<<std::endl;
    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    //std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    //outRegresion << std::setprecision(6) << std::fixed;

    int contLin=maxLine;

    std::cout <<"contLin="<<A1.rows()<<std::endl;

    std::cout <<"contLin2"<<B2.rows()<<std::endl;

    MatrixXd readingB= MatrixXd::Zero((B2.rows()),B2.cols());
    //for (int h=0; h<( maxLine-10);h++){
    //for (int h=0; h<B2.rows() ; h++){

        //readingB.row(int(h+offset)) = B2.row(h);
        //readingB.row(h) = B2.row(h);
        //std::cout <<"h==="<<h<<std::endl;
    //}
    readingB=B2.block(0,0,B2.rows(),B2.cols());
    std::cout <<"fin bucle h"<<std::endl;

    //mediaSerieA = A1.col(1).mean();
    double mediaSerieA0 = A1.col(1).mean();
    double mediaSerieA1 = A1.col(2).mean();
    double mediaSerieA2 = A1.col(3).mean();

    //std::cout <<"	mediaSerieA="<<	round(mediaSerieA0)<<std::endl;

    //mediaSerieB = B2.col(1).mean();
    double mediaSerieB0 = B2.col(1).mean();
    double mediaSerieB1 = B2.col(2).mean();
    double mediaSerieB2 = B2.col(3).mean();

    //std::cout <<"	mediaSerieB="<<	round(mediaSerieB0)<<std::endl;
//Serie
    VectorXd centeredSA,centeredSB,totalCenteredS;
        //for (int i=0; i < contLin ; i++){
            //centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

        //}
    VectorXd centeredSA0 = A1.col(1).array() - mediaSerieA0;
    VectorXd centeredSA1 = A1.col(2).array() - mediaSerieA1;
    VectorXd centeredSA2 = A1.col(3).array() - mediaSerieA2;
    //centeredSA = A1.col(1).array() - mediaSerieA;
    //std::cout <<"	centeredSA0="<<	centeredSA0<<std::endl;
    //centeredSB = B2.col(1).array() - mediaSerieB;
    VectorXd centeredSB0 = B2.col(1).array() - mediaSerieB0;
    VectorXd centeredSB1 = B2.col(2).array() - mediaSerieB1;
    VectorXd centeredSB2 = B2.col(3).array() - mediaSerieB2;
    //std::cout <<"	centeredSB0="<<	centeredSB0<<std::endl;

    double denom=0,denom2=0,sx=0,sy=0,sx0=0,sx1=0,sx2=0,sy0=0,sy1=0,sy2=0;
    for ( int i=0; i < A1.rows(); i++){

            //sx += centeredTA(i)*centeredTA(i);
            //sy += centeredTB(i)*centeredTB(i);
            sx0 += centeredSA0(i)*centeredSA0(i);
            sx1 += centeredSA1(i)*centeredSA1(i);
            sx2 += centeredSA2(i)*centeredSA2(i);
    }
    for ( int i=0; i < B2.rows(); i++){
            sy0 += centeredSB0(i)*centeredSB0(i);
            sy1 += centeredSB1(i)*centeredSB1(i);
            sy2 += centeredSB2(i)*centeredSB2(i);



    }

    denom = sqrt((sx0+sx1+sx2)*(sy0+sy1+sy2));
    double denom2x = sqrt(sx0*sy0);
    double denom2y = sqrt(sx1*sy1);
    double denom2z = sqrt(sx2*sy2);

    // Calculate the correlation series
    double sxy0=0,sxy1=0,sxy2=0, rMax=0;
    double delayMax=0;
    int i=0,j = 0;

    std::cout <<"contLin3="<<contLin<<std::endl;
    std::cout <<"maxLine="<<maxLine<<std::endl;
    int maxRowsA = A1.rows();
    int maxRowsB = B2.rows();
    int maxRows =0;
    if (maxRowsA < maxRowsB){
        maxRows= maxRowsA;
    } else maxRows= maxRowsB;

   // while (i < maxRows && j < maxRows){
    for (int i=0;i<maxRows;i++) {

                   //j = i + offset;
                   j=i;
                   //std::cout <<"i="<<i<<std::endl;
                   if (B2.row(j)(0) < A1.row(i)(0) || B2.row(j)(0) > A1.row(i)(0))
                     //int x=0;
                     continue;
                   else
                      {
                      //sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
                       //sxy += centeredTA[i] * centeredTB[j];
                       //sxy += centeredSA[i] * centeredSB[j];
                       //sxy0 += ((A1.row(i))(1) - mediaSerieA0) * ((readingB.row(j))(1) - mediaSerieB0);
                       //sxy1 += ((A1.row(i))(2) - mediaSerieA1) * ((readingB.row(j))(2) - mediaSerieB1);
                       //sxy2 += ((A1.row(i))(3) - mediaSerieA2) * ((readingB.row(j))(3) - mediaSerieB2);
                       sxy0 += ((A1.row(i))(1) - mediaSerieA0) * ((B2.row(j))(1) - mediaSerieB0);
                       sxy1 += ((A1.row(i))(2) - mediaSerieA1) * ((B2.row(j))(2) - mediaSerieB1);
                       sxy2 += ((A1.row(i))(3) - mediaSerieA2) * ((B2.row(j))(3) - mediaSerieB2);
                       //sx += centeredSA(i)*centeredSA(i);
                       //sy += centeredSB(j)*centeredSB(j);
                    }

                   //i++;
                   //std::cout <<"i2="<<i<<std::endl;

       }


              double r = (sxy0+sxy1+sxy2) / denom;
              double rx = sxy0/denom2x;
              double ry = sxy1/denom2y;
              double rz = sxy2/denom2z;
              //double r = sxy / sqrt(sx*sy);
              double r2 = (rx+ry+rz)/3;

              // r is the correlation coefficient at "delay"
              std::cout <<"r ="<<fabs(r)<<std::endl;
              std::cout <<"r2 ="<<fabs(r2)<<std::endl;
              //std::cout <<"delay ="<<offset<<std::endl;
//              if ((fabs(r) <= 1 )&& (fabs(r) > rMax)){

//                  rMax=r;
//                  delayMax=delay;

//              }
              if (r > r2 )
               return r;
              else
               return r2;


}

double AjusteTiempo::calculateOffsetTXYZ5(int maxLine,  MatrixXd A1,MatrixXd B2){
    //Calculate Correlation for 2 matrix, each has 3 columns and maxLine rows. Use with 3d datasets
    //offset: defines gap between second matrix and first matrix
    //maxLine: number of lines
    //interval: defines variation to calculate offset. If interval = 100, then calculate offset from -100 to 100
    Interpolator myInterpolator;
    std::cout<<" CALCULATE OFFSET TXYZ5......................................."<<std::endl;
    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    //outRegresion << std::setprecision(6) << std::fixed;

    int contLin=maxLine;

    std::cout <<"contLin="<<A1.rows()<<std::endl;

    std::cout <<"contLin2"<<B2.rows()<<std::endl;

    MatrixXd readingB= MatrixXd::Zero((B2.rows()),B2.cols());
    //for (int h=0; h<( maxLine-10);h++){
    //for (int h=0; h<B2.rows() ; h++){

        //readingB.row(int(h+offset)) = B2.row(h);
        //readingB.row(h) = B2.row(h);
        //std::cout <<"h==="<<h<<std::endl;
    //}
    readingB=B2.block(0,0,B2.rows(),B2.cols());
    std::cout <<"fin bucle h"<<std::endl;

    //mediaSerieA = A1.col(1).mean();
    double mediaSerieA0 = A1.col(1).mean();
    double mediaSerieA1 = A1.col(2).mean();
    double mediaSerieA2 = A1.col(3).mean();

    //std::cout <<"	mediaSerieA="<<	round(mediaSerieA0)<<std::endl;

    //mediaSerieB = B2.col(1).mean();
    double mediaSerieB0 = B2.col(1).mean();
    double mediaSerieB1 = B2.col(2).mean();
    double mediaSerieB2 = B2.col(3).mean();

    //std::cout <<"	mediaSerieB="<<	round(mediaSerieB0)<<std::endl;
//Serie
    VectorXd centeredSA,centeredSB,totalCenteredS;
        //for (int i=0; i < contLin ; i++){
            //centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

        //}
    VectorXd centeredSA0 = A1.col(1).array() - mediaSerieA0;
    VectorXd centeredSA1 = A1.col(2).array() - mediaSerieA1;
    VectorXd centeredSA2 = A1.col(3).array() - mediaSerieA2;
    //centeredSA = A1.col(1).array() - mediaSerieA;
    //std::cout <<"	centeredSA0="<<	centeredSA0<<std::endl;
    //centeredSB = B2.col(1).array() - mediaSerieB;
    VectorXd centeredSB0 = B2.col(1).array() - mediaSerieB0;
    VectorXd centeredSB1 = B2.col(2).array() - mediaSerieB1;
    VectorXd centeredSB2 = B2.col(3).array() - mediaSerieB2;
    //std::cout <<"	centeredSB0="<<	centeredSB0<<std::endl;

    double denom=0,denom2=0,sx=0,sy=0,sx0=0,sx1=0,sx2=0,sy0=0,sy1=0,sy2=0;
    for ( int i=0; i < A1.rows(); i++){

            //sx += centeredTA(i)*centeredTA(i);
            //sy += centeredTB(i)*centeredTB(i);
            sx0 += centeredSA0(i)*centeredSA0(i);
            sx1 += centeredSA1(i)*centeredSA1(i);
            sx2 += centeredSA2(i)*centeredSA2(i);
    }
    for ( int i=0; i < B2.rows(); i++){
            sy0 += centeredSB0(i)*centeredSB0(i);
            sy1 += centeredSB1(i)*centeredSB1(i);
            sy2 += centeredSB2(i)*centeredSB2(i);



    }

    denom = sqrt((sx0+sx1+sx2)*(sy0+sy1+sy2));
    double denom2x = sqrt(sx0*sy0);
    double denom2y = sqrt(sx1*sy1);
    double denom2z = sqrt(sx2*sy2);

    // Calculate the correlation series
    double sxy0=0,sxy1=0,sxy2=0, rMax=0;
    double delayMax=0;
    int i=0,j = 0;

    std::cout <<"contLin3="<<contLin<<std::endl;
    std::cout <<"maxLine="<<maxLine<<std::endl;
    int maxRowsA = A1.rows();
    int maxRowsB = B2.rows();
    int maxRows =0;
    if (maxRowsA < maxRowsB){
        maxRows= maxRowsA;
    } else maxRows= maxRowsB;

    while (i < maxRows && j < maxRows){
   // for (int i=0;i<maxRows;i++) {

                   //j = i + offset;
                   //j=i;
                   //std::cout <<"i="<<i<<std::endl;
                   //if (B2.row(j)(0) < A1.row(i)(0)){  //compari timestamps
                   if (myInterpolator.timeLessThan(B2.row(j)(0) , A1.row(i)(0))){  //compari timestamps
                     j++;
                     continue;
                   //}else if (B2.row(j)(0) > A1.row(i)(0)){
                     }else if (myInterpolator.timeGreaterThan(B2.row(j)(0), A1.row(i)(0))){
                       i++;
                       continue;
                    //} else if (B2.row(j)(0) == A1.row(i)(0)){
                       } else if (myInterpolator.timeEqualThan(B2.row(j)(0) ,A1.row(i)(0))){


                        sxy0 += ((A1.row(i))(1) - mediaSerieA0) * ((B2.row(j))(1) - mediaSerieB0);
                        sxy1 += ((A1.row(i))(2) - mediaSerieA1) * ((B2.row(j))(2) - mediaSerieB1);
                        sxy2 += ((A1.row(i))(3) - mediaSerieA2) * ((B2.row(j))(3) - mediaSerieB2);
                        j++;
                        i++;

                     }






                   //i++;
                   //std::cout <<"i2="<<i<<std::endl;

       }


              double r = (sxy0+sxy1+sxy2) / denom;
              double rx = sxy0/denom2x;
              double ry = sxy1/denom2y;
              double rz = sxy2/denom2z;
              //double r = sxy / sqrt(sx*sy);
              double r2 = (rx+ry+rz)/3;
              r= fabs(r);
              r2=fabs(r2);
              // r is the correlation coefficient at "delay"
              std::cout <<"r ="<<r<<std::endl;
              std::cout <<"r2 ="<<r2<<std::endl;
              //std::cout <<"delay ="<<offset<<std::endl;
//              if ((fabs(r) <= 1 )&& (fabs(r) > rMax)){

//                  rMax=r;
//                  delayMax=delay;

//              }
              if (r > r2 )

               return r;
              else
               return r2;


}

double AjusteTiempo::calculateOffsetTXYZ6(int maxLine,  MatrixXd A1,MatrixXd B2){
    //Calculate Normalized Cross Correlation for 2 matrix, each has 3 columns and maxLine rows. Use with 3d datasets
    //offset: defines gap between second matrix and first matrix
    //maxLine: number of lines
    //interval: defines variation to calculate offset. If interval = 100, then calculate offset from -100 to 100
    Interpolator myInterpolator;
    MatrixXd time_Distance3DA1;
    MatrixXd time_Distance3DB2;
    //this->calculateDistanceForDataSet3D( A1, time_Distance3DA1 );
    int rows = A1.rows();
    double t=0,x=0, y=0, z=0;
    time_Distance3DA1 = MatrixXd::Zero(rows,2);
    for (int i=0; i < rows; i++){
        t=A1.row(i)(0);
        x=A1.row(i)(1);
        y=A1.row(i)(2);
        z=A1.row(i)(3);
        // the Distance to the origin (0,0,0) of a 3d point (x,y, z)
        // should be calculated like sqrt( (0-x)^2 + (0-y)^2 + (0-z)^2)
        time_Distance3DA1.row(i)<< t, sqrt( x*x + y*y + z*z );
    }
    //this->calculateDistanceForDataSet3D( B2, time_Distance3DB2 );
    rows = B2.rows();

    time_Distance3DB2 = MatrixXd::Zero(rows,2);
    for (int i=0; i < rows; i++){
        t=B2.row(i)(0);
        x=B2.row(i)(1);
        y=B2.row(i)(2);
        z=B2.row(i)(3);
        // the Distance to the origin (0,0,0) of a 3d point (x,y, z)
        // should be calculated like sqrt( (0-x)^2 + (0-y)^2 + (0-z)^2)
        time_Distance3DB2.row(i)<< t, sqrt( x*x + y*y + z*z );
    }
    std::cout<<"time_Distance3DA1.rows()"<<time_Distance3DA1.rows()<<"\n";
    std::cout<<"time_Distance3DB2.rows()"<<time_Distance3DB2.rows()<<"\n";
    std::cout<<" CALCULATE OFFSET TXYZ6.with normalized cross correlation......................................"<<std::endl;

//    std::cout << std::setprecision(6) << std::fixed;
//    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaDistanceA.txt" );
//    outA << std::setprecision(6) << std::fixed;
//    for (int i=0;i++;i < time_Distance3DA1.rows()){
//        outA<< time_Distance3DA1(0) << " " << time_Distance3DA1(1) << std::endl;
//    }

//    outA.close();


//    std::cout << std::setprecision(6) << std::fixed;
//    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaDistanceB.txt" );
//    outB << std::setprecision(6) << std::fixed;
//    for (int i=0;i++;i < time_Distance3DB2.rows()){
//        outB<< time_Distance3DB2(0) << " " << time_Distance3DB2(1) << std::endl;
//    }

//    outB.close();

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    //outRegresion << std::setprecision(6) << std::fixed;

    int contLin=maxLine;

    std::cout <<"contLin="<<A1.rows()<<std::endl;

    std::cout <<"contLin2"<<B2.rows()<<std::endl;

    MatrixXd readingB= MatrixXd::Zero((B2.rows()),B2.cols());
    //for (int h=0; h<( maxLine-10);h++){
    //for (int h=0; h<B2.rows() ; h++){

        //readingB.row(int(h+offset)) = B2.row(h);
        //readingB.row(h) = B2.row(h);
        //std::cout <<"h==="<<h<<std::endl;
    //}
    readingB=B2.block(0,0,B2.rows(),B2.cols());
    std::cout <<"fin bucle h"<<std::endl;

    //mediaSerieA = A1.col(1).mean();
    double mediaSerieA = A1.col(1).mean();

    //std::cout <<"	mediaSerieA="<<	round(mediaSerieA0)<<std::endl;

    //mediaSerieB = B2.col(1).mean();
    double mediaSerieB = B2.col(1).mean();

    //std::cout <<"	mediaSerieB="<<	round(mediaSerieB0)<<std::endl;
//Serie
    VectorXd centeredSA,centeredSB,totalCenteredS;
        //for (int i=0; i < contLin ; i++){
            //centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

        //}
    centeredSA = A1.col(1).array() - mediaSerieA;
    //centeredSA = A1.col(1).array() - mediaSerieA;
    //std::cout <<"	centeredSA0="<<	centeredSA0<<std::endl;
    //centeredSB = B2.col(1).array() - mediaSerieB;
    centeredSB = B2.col(1).array() - mediaSerieB;
    //std::cout <<"	centeredSB0="<<	centeredSB0<<std::endl;

    double denom=0,denom2=0,sx=0,sy=0,sx0=0,sx1=0,sx2=0,sy0=0,sy1=0,sy2=0;
    for ( int i=0; i < A1.rows(); i++){

            //sx += centeredTA(i)*centeredTA(i);
            //sy += centeredTB(i)*centeredTB(i);
            sx += centeredSA(i)*centeredSA(i);
    }

    for ( int i=0; i < B2.rows(); i++){
            sy += centeredSB(i)*centeredSB(i);



    }

    denom = sqrt((sx)*(sy));

    // Calculate the correlation series
    double sxy=0, rMax=0;
    double delayMax=0;
    int i=0,j = 0;

    std::cout <<"contLin3="<<contLin<<std::endl;
    std::cout <<"maxLine="<<maxLine<<std::endl;
    int maxRowsA = A1.rows();
    int maxRowsB = B2.rows();
    int maxRows =0;
    if (maxRowsA < maxRowsB){
        maxRows= maxRowsA;
    } else maxRows= maxRowsB;

    while (i < maxRows && j < maxRows){
   // for (int i=0;i<maxRows;i++) {

                   //j = i + offset;
                   //j=i;
                   //std::cout <<"i="<<i<<std::endl;
                   //if (B2.row(j)(0) < A1.row(i)(0)){  //compari timestamps
                   if (myInterpolator.timeLessThan(B2.row(j)(0) , A1.row(i)(0))){  //comparing timestamps
                     j++;
                     continue;
                   //}else if (B2.row(j)(0) > A1.row(i)(0)){
                     }else if (myInterpolator.timeGreaterThan(B2.row(j)(0), A1.row(i)(0))){
                       i++;
                       continue;
                    //} else if (B2.row(j)(0) == A1.row(i)(0)){
                       } else if (myInterpolator.timeEqualThan(B2.row(j)(0) ,A1.row(i)(0))){


                        sxy += ((A1.row(i))(1) - mediaSerieA) * ((B2.row(j))(1) - mediaSerieB);

                        j++;
                        i++;

                     }






                   //i++;
                   //std::cout <<"i2="<<i<<std::endl;

       }


              double r = (sxy) / denom;
              //double r = sxy / sqrt(sx*sy);

              r= fabs(r);

              // r is the correlation coefficient at "delay"
//              std::cout <<"r ="<<r<<std::endl;

//              //std::cout <<"delay ="<<offset<<std::endl;
//              if ((fabs(r) <= 1 )&& (fabs(r) > rMax)){

//                  rMax=r;
//                 // delayMax=delay;

//              }
//              return rMax;
                return r;
}








void AjusteTiempo::calculateOffset( int maxLine, int intervalo,double offset, MatrixXd& A1, MatrixXd& B2)

{
	// offset: define la separación de la segunda serie que sera una serie artificial. La segunda serie S2 = S1 + offset
	// intervalo: define la amplitud de la ventana desde la que buscaremos la correlación
	// maxLine : numero máximo de líneas


	//inputFile >> std::setprecision(6) >> std::fixed;

	std::cout<<" CALCULATE OFFSET ......................................."<<std::endl;
    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outA( "/home/tfm3/workspace/AjusteTiempo/miSalidaA.txt" );
    outA << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outB( "/home/tfm3/workspace/AjusteTiempo/miSalidaB.txt" );
    outB << std::setprecision(6) << std::fixed;

    std::cout << std::setprecision(6) << std::fixed;
    std::ofstream outRegresion( "/home/tfm3/workspace/AjusteTiempo/miSalidaRegresion.txt" );
    outRegresion << std::setprecision(6) << std::fixed;

	std::cout <<"antes de declarar la variable"<<std::endl;
	//int maxLine = 30;
	maxLine=maxLine;
	MatrixXd readingA (maxLine,3);//size is big enough to store an unknown number of data rows from file
	int totalLines = B2.rows()+offset;
	MatrixXd readingB= MatrixXd::Zero(totalLines,B2.cols());
	for (int h=0; h< maxLine;h++){
		//std::cout <<"h="<<h<<std::endl;
		readingB.row(int(h+offset)) = B2.row(h);
	}
	//VectorXd vTiempoA (maxLine), vSerieA (maxLine) ;
	//VectorXd vTiempoB (int(maxLine+offset)), vSerieB (int(maxLine+offset)) ;
	//read a file
	int contLin=0;
	double timestamp,rx,ry,rz,q1,q2,q3,q4=0;
	double mediaTiempoA, mediaTiempoB, mediaSerieA, mediaSerieB;
	std::cout <<"antes de leer el archivo"<<std::endl;
	//int miOffset=5;
    double miOffset = offset;
    double newContLin = 0;
    std::cout <<"miOffset="<<miOffset<<std::endl;

	//while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
    double aleatorio =0, valueToB=0, valueToA=0;
	//while (  contLin<maxLine ){
	//	   std::cout <<"contLin="<<contLin<<std::endl;


	//std::cout <<"fin de bucle"<<contLin<<std::endl;

	contLin=maxLine;
	std::cout <<"contLin="<<contLin<<std::endl;
	//Calcular autoCorrelacion
	//MatrixXd A (contLin,3);
	//MatrixXd B (contLin,3);
	//std::cout <<"contLin1"<<contLin<<std::endl;
	//VectorXd vSerieA,vSerieB ;
	//vSerieA << A1.col(1).array();
	//vSerieA = A1.block(0,1,contLin,1);
	//std::cout <<"contLin1.5"<<contLin<<std::endl;
	//VectorXd
	//vSerieB = B2.block(0,1,contLin,1);

	//vSerieB = B2.col(1).array();
	std::cout <<"contLin2"<<contLin<<std::endl;

	//mediaSerieA = A1.col(1).mean();
	mediaSerieA = A1.col(0).mean();
	//if ( abs(mediaSerieA )< 0.000001){
	//		mediaSerieA = 0.0;
	//}
	std::cout <<"	mediaSerieA="<<	round(mediaSerieA)<<std::endl;

	//mediaSerieB = B2.col(1).mean();
	mediaSerieB = B2.col(0).mean();

	//mediaSerieB = B2.col(1).mean();
	//if ( abs(mediaSerieB )< 0.000001)
	//			mediaSerieB = 0.0;
	std::cout <<"	mediaSerieB="<<	round(mediaSerieB)<<std::endl;


	//Serie
	VectorXd centeredSA,centeredSB,totalCenteredS;
		//for (int i=0; i < contLin ; i++){
			//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

		//}
	centeredSA = A1.col(0).array() - mediaSerieA;
	//centeredSA = A1.col(1).array() - mediaSerieA;
	//std::cout <<"	centeredSA="<<	centeredSA<<std::endl;
	//centeredSB = B2.col(1).array() - mediaSerieB;
	centeredSB = B2.col(0).array() - mediaSerieB;
	//std::cout <<"	centeredSB="<<	centeredSB<<std::endl;

	double denom=0,sx=0,sy=0;
	for ( int i=0; i < contLin; i++){
			//sx += centeredTA(i)*centeredTA(i);
			//sy += centeredTB(i)*centeredTB(i);
			sx += centeredSA(i)*centeredSA(i);
			sy += centeredSB(i)*centeredSB(i);
	}

	denom = sqrt(sx*sy);

	// Calculate the correlation series
       double sxy=0, rMax=0;
       int j = 0,delayMax=0;

       std::cout <<"contLin3="<<contLin<<std::endl;
       std::cout <<"maxLine="<<maxLine<<std::endl;

	  for (int delay=-intervalo;delay<intervalo;delay++) {
		  sxy = 0;
		  sx=0;
		  sy=0;
		  //std::cout <<"delay ="<<delay<<std::endl;
		  //for (int i=0;i<contLin;i++) {
		  for (int i=0;i<maxLine;i++) {
			 //std::cout <<"i ="<<i<<" j="<<j<<std::endl;
			 j = i + delay;
			 if (j < 0 || j >= maxLine)
				continue;
			 else
				//sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
				 //sxy += centeredTA[i] * centeredTB[j];
				 //sxy += centeredSA[i] * centeredSB[j];
				 //sxy += ((A1.row(i))(1) - mediaSerieA) * ((readingB.row(j))(1) - mediaSerieB);
				 sxy += ((A1.row(i))(0) - mediaSerieA) * ((readingB.row(j))(0) - mediaSerieB);
                 //sx += centeredSA(i)*centeredSA(i);
			     //sy += centeredSB(j)*centeredSB(j);
		  }



		  double r = sxy / denom;
		  //double r = sxy / sqrt(sx*sy);
          if ((fabs(r) <= 1 )&& (fabs(r) > rMax)){
        	  rMax=r;
        	  delayMax=delay;
          }
	      // r is the correlation coefficient at "delay"
		 //std::cout <<"r ="<<fabs(r)<<std::endl;
         //std::cout <<"delay ="<<delay<<std::endl;
         outRegresion<< r << " " << delay << std::endl;
		  //std::cout <<"i="<<i<<std::endl;
	   }
	  std::cout <<"rMax="<<rMax<<std::endl;
	  std::cout <<"delayMax="<<delayMax<<std::endl;
	  outRegresion.close();


}


double AjusteTiempo::calcularAutocorrelacion4(int maxLine, int intervalo,double delay, MatrixXd& A, MatrixXd& B)

{
	//-2018abr18
	// This will be used with interpolation, calculate regression for 2 series
	// offset: Defines how to separate de second serie. B + offset
	// intervalo: defines la amplitud de la ventana desde la que buscaremos la correlación
	// maxLine : numero máximo de líneas
	// MatrixXd A and B is type ( time,x,y,z)





	double mediaTiempoA, mediaTiempoB, meanXSerieA, meanXSerieB;
	std::cout <<"Inside calcularAutocorrelacion4"<<std::endl;
	//int miOffset=5;
    //double miOffset = offset;
    double newContLin = 0;
    //std::cout <<"miOffset="<<miOffset<<std::endl;
    // initialize random seed:
    srand (time(NULL));
	//while ( (infile >> timestamp >> rx >> ry >> rz >> q1 >> q2 >> q3 >> q4) && contLin<maxLine ){
    double aleatorio =0, valueToB=0;


	//contLin=maxLine;
	//std::cout <<"contLin="<<contLin<<std::endl;
	//Calcular autoCorrelacion

	meanXSerieA = A.row(1).mean(); // Takes the xA coordinate
	//std::cout <<"	meanXSerieA="<<	meanXSerieA<<std::endl;

	meanXSerieB = B.row(1).mean(); // takes the xB coordinate
	//std::cout <<"	meanXSerieB="<<	meanXSerieB<<std::endl;


	//Serie
	VectorXd centeredSA,centeredSB,totalCenteredS;
		//for (int i=0; i < contLin ; i++){
			//centeredTA.row(contLin)<< vTiempoA.array() - mediaTiempoA;

		//}
	//centeredSA = A.row(1).array() - meanXSerieA;
	centeredSA = A.col(1).array() - meanXSerieA;
	//std::cout <<"	centeredSA="<<	centeredSA<<std::endl;

	centeredSB = B.col(1).array() - meanXSerieB;
	//std::cout <<"	centeredSB="<<	centeredSB<<std::endl;

	double denom=0,sx=0,sy=0;
	std::cout <<"Inside calcularAutocorrelacion4:Antes BucleA"<<std::endl;
	for ( int i=0; i < A.rows(); i++){
			//sx += centeredTA(i)*centeredTA(i);
			//sy += centeredTB(i)*centeredTB(i);
		sx += centeredSA(i)*centeredSA(i);

	}
	std::cout <<"Inside calcularAutocorrelacion4:Antes BucleB"<<std::endl;
	for (int i=0; i< B.rows(); i ++){
		sy += centeredSB(i)*centeredSB(i);
	}

	denom = sqrt(sx*sy);

	// Calculate the correlation
	std::cout <<"Inside calcularAutocorrelacion4:Antes de empezar a calcular la correlacion"<<std::endl;
       double sxy=0, rMax=0;
       int j = 0,delayMax=0;

       std::cout <<"Inside calcularAutocorrelacion4:A.rows()="<<A.rows()<<std::endl;
       std::cout <<"Inside calcularAutocorrelacion4:B.rows()="<<B.rows()<<std::endl;
	  sxy = 0;
	  sx=0;
	  sy=0;
	  int maxRowsA = A.rows();
	  int maxRowsB = B.rows();
	  //for (int i=0;i<maxRows;i++) {
	  int i =0;

	  while (i < maxRowsA && j < maxRowsB){


		 j = i + delay;
		 if (j < 0 || j >= maxRowsB)
			continue;
		 else
			//sxy += (vTiempoA[i] - mx) * (vTiempoB[j] - my);
			 //sxy += centeredTA[i] * centeredTB[j];
			 //sxy += centeredSA[i] * centeredSB[j];
			 sxy += ((A.row(i))(1) - meanXSerieA) * ((B.row(j))(1) - meanXSerieB);
			 //sx += centeredSA(i)*centeredSA(i);
			 //sy += centeredSB(j)*centeredSB(j);
		 i++;
	  }



	  double r = sxy / denom;
	  //double r = sxy / sqrt(sx*sy);

	  // r is the correlation coefficient at "delay"
	  std::cout <<"r ="<<fabs(r)<<std::endl;
	  std::cout <<"delay ="<<delay<<std::endl;
      return r;
	  //std::cout <<"i="<<i<<std::endl;




}





/*

int main( int argc, char** argv )
{
	//std::cout << std::setprecision(6) << std::fixed;




		std::cout <<"	ESTOY EN BLOQUE 2="<<std::endl;
        AjusteTiempo micorrelador;
		MatrixXd A,B;
		double offset = 0;
		int maxLine = 2500;
		int intervalo = 200;		//micorrelador.calcularAutocorrelacion( maxLine,intervalo, offset, A,  B);
		//micorrelador.calcularAutocorrelacion2( maxLine,intervalo, offset, A,  B);
		micorrelador.calculateOffset( maxLine,intervalo, offset, A,  B);
		//micorrelador.calcularAutocorrelacion3( 'z',maxLine,intervalo, offset, A,  B);
		//micorrelador.calcularAutocorrelacion3( 'y',maxLine,intervalo, offset, A,  B);
		//micorrelador.calcularAutocorrelacion3( 'z',maxLine,intervalo, offset, A,  B);




}
*/



//create sequence 1, two variables , T (time), X

