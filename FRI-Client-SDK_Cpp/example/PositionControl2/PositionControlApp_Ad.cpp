#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h> // strstr
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;

#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"  //ip address set for KONI connection
#define SHM_SIZE 1024

FILE *NewDataFile(void);


int *MakeFloatSharedMemory(int HowBig);
float *MakeFloatSharedMemory2(int HowBig2);

// Main
int main(int argc, char** argv)
{
	FILE *OutputFile;
	FILE *fp;

	double al = 0.0005;
	double ftx;
	double fty;
	double ftx_un;
	double fty_un;
	double zerox = 0;
	double zeroy = 0;
	double ftx_0 = 0.0;
	double fty_0 = 0.0;
	int *data;
	data = MakeFloatSharedMemory(2);
	int firstIt = 0;
	int count = 0;
	int state = 0;

	double phi_euler = 0;
	double theta_euler = 0;
	double psi_euler = 0;

	float sampletime = 0;
	double MJoint[7] = { 0 };
	double ETorque[7] = { 0 };
	double MaxRadPerSec[7] = { 1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159 }; // Max velocity
	double MaxRadPerStep[7] = { 0 };//will be calculated
	double MaxJointLimitRad[7] = { 2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543 };//Max joint limits
	double MinJointLimitRad[7] = { -2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543 }; //Min joint limits
	double FirstPositionDelta[7] = { 0.0175,0.0175,0.0175,0.0175,0.0175,0.0175,0.0175 }; //maximum deviation from initial position in trajectory from start position in robot(radians)

	MatrixXd x_0(6, 1); x_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_00(6, 1); x_00 << 0, 0, 0, 0, 0, 0;
	MatrixXd q_0(6, 1); q_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_e(6, 1); x_e << 0, 0, 0, 0, 0, 0;
	MatrixXd force(6, 1); force << 0, 0, 0, 0, 0, 0;
	MatrixXd q_new(6, 1); q_new << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new(6, 1); x_new << 0, 0, 0, 0, 0, 0;

	MatrixXd torques_0(6, 1); torques_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd t_e(6, 1); t_e << 0, 0, 0, 0, 0, 0;

	// DH Parameter----------------------------------------------------
	MatrixXd alpha(1, 7); alpha << M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, 0;
	MatrixXd a(1, 7); a << 0, 0, 0, 0, 0, 0, 0;
	MatrixXd d(1, 7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
	MatrixXd theta(1, 7); theta << 0, 0, 0, 0, 0, 0, 0;

	/*MatrixXd q_int(6, 1); q_int.setZero(6, 1);
	q_int(0) = -1.779862;;
	q_int(1) = 0.821814;
	q_int(2) = -0.067855;
	q_int(3) = 1.302481;
	q_int(4) = 0.284275;
	q_int(5) = -1.118251;*/
	

	//??
	struct timespec start2, finish2;
	struct timeval start, end;
	long mtime, seconds, useconds;
	gettimeofday(&start, NULL);
	//??

	if (argc < 1)
	{
		printf(
			"\nKUKA LBR Position Control application\n\n"
			"\tCommand line arguments:\n"
			"\t1) filename of trajectory file"
			"\t2) remote hostname (optional)\n"
			"\t3) port ID (optional)\n"
		);
		return 1;
	}

	char* filename = argv[1]; //Requiered command line argument for the file name for trajectory file
	const char* hostname = (argc >= 3) ? argv[2] : DEFAULT_IP; //optional command line argument for ip address (default is for KONI)
	int port = (argc >= 4) ? atoi(argv[3]) : DEFAULT_PORTID; //optional comand line argument for port
	char path[4096];


	sampletime = 0.001;
	fprintf(stdout, "Sample Time:%f seconds\n", sampletime);

	//calculate max step value
	for (int i = 0; i<7; i++)
	{
		MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
	}

	//******* Client Configuration********//----------------------------------------------------------------

	// create new joint position client
	PositionControlClient client;
	client.intvalues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);




	// create new udp connection
	UdpConnection connection;


	// pass connection and client to a new FRI client application
	ClientApplication app(connection, client);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);
	//------------------------------------------------------------------------------------------

	//create file for output
	OutputFile = NewDataFile();

	memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7);

	//Sending initial joint angels
	client.NextJoint[0] = MJoint[0];
	client.NextJoint[1] = MJoint[1];
	client.NextJoint[2] = MJoint[2];
	client.NextJoint[3] = MJoint[3];
	client.NextJoint[4] = MJoint[4];
	client.NextJoint[5] = MJoint[5];
	//client.NextJoint[6] = -0.7854;
	client.NextJoint[6] = MJoint[6];
	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));

	if (state < 2000)
	{
		zerox = 100 * al*(double)data[0] / 1000000 + (1 - 100 * al)*zerox;
		zeroy = 100 * al*(double)data[1] / 1000000 + (1 - 100 * al)*zeroy;
		state++;
	}

	while (1)
	{
		clock_gettime(CLOCK_MONOTONIC, &start2);
		//---------------------------timestamp----------------------------------
		gettimeofday(&end, NULL);
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		//----------------------------------------------------------------------
		
		if (count == 1)//first time inside
		{
			sampletime = client.GetTimeStep();
			for (int i = 0; i < 7; i++)
			{
				client.MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
			}
			count++;
		}


		// Get the data force and joint angles
		memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7);
		memcpy(ETorque, client.GetExtTor(), sizeof(double) * 7);


		// Forward Kinematic------------------------------------------------------------------------

		theta << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6];

		MatrixXd A1(4, 4); A1 << cos(theta(0, 0)), -sin(theta(0, 0))*cos(alpha(0, 0)), sin(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*cos(theta(0, 0)),
			sin(theta(0, 0)), cos(theta(0, 0))*cos(alpha(0, 0)), -cos(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*sin(theta(0, 0)),
			0, sin(alpha(0, 0)), cos(alpha(0, 0)), d(0, 0),
			0, 0, 0, 1;
		MatrixXd A2(4, 4); A2 << cos(theta(0, 1)), -sin(theta(0, 1))*cos(alpha(0, 1)), sin(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*cos(theta(0, 1)),
			sin(theta(0, 1)), cos(theta(0, 1))*cos(alpha(0, 1)), -cos(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*sin(theta(0, 1)),
			0, sin(alpha(0, 1)), cos(alpha(0, 1)), d(0, 1),
			0, 0, 0, 1;
		MatrixXd A3(4, 4); A3 << cos(theta(0, 2)), -sin(theta(0, 2))*cos(alpha(0, 2)), sin(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*cos(theta(0, 2)),
			sin(theta(0, 2)), cos(theta(0, 2))*cos(alpha(0, 2)), -cos(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*sin(theta(0, 2)),
			0, sin(alpha(0, 2)), cos(alpha(0, 2)), d(0, 2),
			0, 0, 0, 1;
		MatrixXd A4(4, 4); A4 << cos(theta(0, 3)), -sin(theta(0, 3))*cos(alpha(0, 3)), sin(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*cos(theta(0, 3)),
			sin(theta(0, 3)), cos(theta(0, 3))*cos(alpha(0, 3)), -cos(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*sin(theta(0, 3)),
			0, sin(alpha(0, 3)), cos(alpha(0, 3)), d(0, 3),
			0, 0, 0, 1;
		MatrixXd A5(4, 4); A5 << cos(theta(0, 4)), -sin(theta(0, 4))*cos(alpha(0, 4)), sin(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*cos(theta(0, 4)),
			sin(theta(0, 4)), cos(theta(0, 4))*cos(alpha(0, 4)), -cos(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*sin(theta(0, 4)),
			0, sin(alpha(0, 4)), cos(alpha(0, 4)), d(0, 4),
			0, 0, 0, 1;
		MatrixXd A6(4, 4); A6 << cos(theta(0, 5)), -sin(theta(0, 5))*cos(alpha(0, 5)), sin(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*cos(theta(0, 5)),
			sin(theta(0, 5)), cos(theta(0, 5))*cos(alpha(0, 5)), -cos(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*sin(theta(0, 5)),
			0, sin(alpha(0, 5)), cos(alpha(0, 5)), d(0, 5),
			0, 0, 0, 1;

		MatrixXd T01(4, 4); T01 << A1;
		MatrixXd T02(4, 4); T02 << A1*A2;
		MatrixXd T03(4, 4); T03 << A1*A2*A3;
		MatrixXd T04(4, 4); T04 << A1*A2*A3*A4;
		MatrixXd T05(4, 4); T05 << A1*A2*A3*A4*A5;
		MatrixXd T06(4, 4); T06 << A1*A2*A3*A4*A5*A6;

		// Inverse Kinematic----------------------------------------------------------------------

		phi_euler = atan2(T06(1, 2), T06(0, 2));
		theta_euler = atan2(sqrt(pow(T06(1, 2), 2) + pow(T06(0, 2), 2)), T06(2, 2));
		psi_euler = atan2(T06(2, 1), -T06(2, 0));

		MatrixXd z0(3, 1); z0 << 0, 0, 1;
		MatrixXd z1(3, 1); z1 << T01(0, 2), T01(1, 2), T01(2, 2);
		MatrixXd z2(3, 1); z2 << T02(0, 2), T02(1, 2), T02(2, 2);
		MatrixXd z3(3, 1); z3 << T03(0, 2), T03(1, 2), T03(2, 2);
		MatrixXd z4(3, 1); z4 << T04(0, 2), T04(1, 2), T04(2, 2);
		MatrixXd z5(3, 1); z5 << T05(0, 2), T05(1, 2), T05(2, 2);
		MatrixXd z6(3, 1); z6 << T06(0, 2), T06(1, 2), T06(2, 2);

		MatrixXd p0(3, 1); p0 << 0, 0, 0;
		MatrixXd p1(3, 1); p1 << T01(0, 3), T01(1, 3), T01(2, 3);
		MatrixXd p2(3, 1); p2 << T02(0, 3), T02(1, 3), T02(2, 3);
		MatrixXd p3(3, 1); p3 << T03(0, 3), T03(1, 3), T03(2, 3);
		MatrixXd p4(3, 1); p4 << T04(0, 3), T04(1, 3), T04(2, 3);
		MatrixXd p5(3, 1); p5 << T05(0, 3), T05(1, 3), T05(2, 3);
		MatrixXd p6(3, 1); p6 << T06(0, 3), T06(1, 3), T06(2, 3);

		MatrixXd J1(6, 1); J1 << z0(1, 0)*(p6(2, 0) - p0(2, 0)) - z0(2, 0)*(p6(1, 0) - p0(1, 0)),
			-z0(0, 0)*(p6(2, 0) - p0(2, 0)) + z0(2, 0)*(p6(0, 0) - p0(0, 0)),
			z0(0, 0)*(p6(1, 0) - p0(1, 0)) - z0(1, 0)*(p6(0, 0) - p0(0, 0)),
			z0(0, 0), z0(1, 0), z0(2, 0);
		MatrixXd J2(6, 1); J2 << z1(1, 0)*(p6(2, 0) - p1(2, 0)) - z1(2, 0)*(p6(1, 0) - p1(1, 0)),
			-z1(0, 0)*(p6(2, 0) - p1(2, 0)) + z1(2, 0)*(p6(0, 0) - p1(0, 0)),
			z1(0, 0)*(p6(1, 0) - p1(1, 0)) - z1(1, 0)*(p6(0, 0) - p1(0, 0)),
			z1(0, 0), z1(1, 0), z1(2, 0);
		MatrixXd J3(6, 1); J3 << z2(1, 0)*(p6(2, 0) - p2(2, 0)) - z2(2, 0)*(p6(1, 0) - p2(1, 0)),
			-z2(0, 0)*(p6(2, 0) - p2(2, 0)) + z2(2, 0)*(p6(0, 0) - p2(0, 0)),
			z2(0, 0)*(p6(1, 0) - p2(1, 0)) - z2(1, 0)*(p6(0, 0) - p2(0, 0)),
			z2(0, 0), z2(1, 0), z2(2, 0);
		MatrixXd J4(6, 1); J4 << z3(1, 0)*(p6(2, 0) - p3(2, 0)) - z3(2, 0)*(p6(1, 0) - p3(1, 0)),
			-z3(0, 0)*(p6(2, 0) - p3(2, 0)) + z3(2, 0)*(p6(0, 0) - p3(0, 0)),
			z3(0, 0)*(p6(1, 0) - p3(1, 0)) - z3(1, 0)*(p6(0, 0) - p3(0, 0)),
			z3(0, 0), z3(1, 0), z3(2, 0);
		MatrixXd J5(6, 1); J5 << z4(1, 0)*(p6(2, 0) - p4(2, 0)) - z4(2, 0)*(p6(1, 0) - p4(1, 0)),
			-z4(0, 0)*(p6(2, 0) - p4(2, 0)) + z4(2, 0)*(p6(0, 0) - p4(0, 0)),
			z4(0, 0)*(p6(1, 0) - p4(1, 0)) - z4(1, 0)*(p6(0, 0) - p4(0, 0)),
			z4(0, 0), z4(1, 0), z4(2, 0);
		MatrixXd J6(6, 1); J6 << z5(1, 0)*(p6(2, 0) - p5(2, 0)) - z5(2, 0)*(p6(1, 0) - p5(1, 0)),
			-z5(0, 0)*(p6(2, 0) - p5(2, 0)) + z5(2, 0)*(p6(0, 0) - p5(0, 0)),
			z5(0, 0)*(p6(1, 0) - p5(1, 0)) - z5(1, 0)*(p6(0, 0) - p5(0, 0)),
			z5(0, 0), z5(1, 0), z5(2, 0);

		MatrixXd Jg(6, 6); Jg << J1, J2, J3, J4, J5, J6;


		MatrixXd Tphi(6, 6); Tphi << 1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, -sin(phi_euler), cos(phi_euler)*sin(theta_euler),
			0, 0, 0, 0, cos(phi_euler), sin(phi_euler)*sin(theta_euler),
			0, 0, 0, 1, 0, cos(theta_euler);

		MatrixXd Ja(6, 6); Ja << Tphi.inverse()*Jg;
		//-----------------------------------------------------------------------------------

		// Initializing Stiffness Damping and Inertia

		MatrixXd stiffness(6, 6); stiffness << 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0;
		MatrixXd damping(6, 6); damping << 1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 0.5, 0, 0, 0,
			0, 0, 0, 0.5, 0, 0,
			0, 0, 0, 0, 0.5, 0,
			0, 0, 0, 0, 0, 0.5;
		MatrixXd inertia(6, 6); inertia << 1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 100, 0, 0,
			0, 0, 0, 0, 100, 0,
			0, 0, 0, 0, 0, 100;
		//-----------------------------------------------------------------------------------


		MatrixXd torques(6, 1); torques << ETorque[0], ETorque[1], ETorque[2], ETorque[3], ETorque[4], ETorque[5];

		t_e << al*torques + (1 - al)*torques_0;
		torques_0 << t_e;

		if (firstIt == 0)//first time inside
		{
			firstIt = 1;
			x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
			//t_0 << -0.0313991, -0.414115, -0.00613078, -0.450773, 0.265899, -0.0418674;
		}

		ftx = (double)data[0] / 1000000 - zerox;
		ftx_un = (double)data[0] / 1000000 - zerox;

		fty = (double)data[1] / 1000000 - zeroy;
		fty_un = (double)data[1] / 1000000 - zeroy;

		ftx = al*ftx + (1 - al)*ftx_0;
		ftx_0 = ftx;
		fty = al*fty + (1 - al)*fty_0;
		fty_0 = fty;

		force << ftx, fty, 0, 0, 0, 0;


		q_0 << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5];

		x_00 << x_0;
		x_0 << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;

		//x_new << (inertia + damping + stiffness).inverse()*(force + inertia*(x_0 - x_00) + stiffness*(x_e - x_0)) + x_0;
		x_new << (inertia + damping + stiffness).inverse()*(force + inertia*(2 * x_0 - x_00) + damping*x_0 + stiffness*x_e) + x_0;

		if (0.21 < x_new(2) & x_new(2) < 0.51)
		{
			//q_new << Ja.inverse()*(inertia + damping + stiffness).inverse()*(force + inertia*(x_0 - x_00) + stiffness*(x_e - x_0)) + q_0;//+(q_old-q_0)*0.3;
			q_new << Ja.inverse()*(inertia + damping + stiffness).inverse()*(force + inertia*(2 * x_0 - x_00) + damping*x_0 + stiffness*x_e) + q_0;
		}

		client.NextJoint[0] = q_new(0);
		client.NextJoint[1] = q_new(1);
		client.NextJoint[2] = q_new(2);
		client.NextJoint[3] = q_new(3);
		client.NextJoint[4] = q_new(4);
		client.NextJoint[5] = q_new(5);
		client.NextJoint[6] = client.LastJoint[6];

		fprintf(OutputFile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], ftx_un, fty_un, x_new(0), x_new(1), x_new(2), x_new(3), x_new(4), x_new(5));

		
	}

	fclose(OutputFile);
	fprintf(stdout, "File closed.\n\n\n");
	// disconnect from controller

	fprintf(stdout, "Shhh.. I'm sleeping!\n");
	usleep(10000000);//microseconds //wait for close on other side
	app.disconnect();

	gettimeofday(&start, NULL);

	return 1;
}

FILE *NewDataFile(void) //this may be specific to linux OS
{
	FILE *fp;
	time_t rawtime;
	struct tm *timeinfo;
	char namer[40];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(namer, 40, "Output%Y-%j-%H_%M_%S.txt", timeinfo);//creates a file name that has year-julian day-hour min second (unique for each run, no chance of recording over previous data)
	fp = fopen(namer, "w");//open output file
	return fp;
}


// Shared Memory-------------------------------------------------------
int *MakeFloatSharedMemory(int HowBig)
{
	key_t key;
	int shmid;
	int *dataShared;

	dataShared = (int *)malloc(HowBig * sizeof(int));
	/* make the key */
	if ((key = ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile", 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid = shmget(key, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	dataShared = (int *)shmat(shmid, (void *)0, 0);

	if (dataShared == (int *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<HowBig; i++)
	{
		dataShared[i] = 0.0;
	}

	return dataShared;
}


// Shared Memory Display--------------------------------------------------------------
float *MakeFloatSharedMemory2(int HowBig2)
{
	key_t key2;
	int shmid2;
	float *dataShared2;

	dataShared2 = (float *)malloc(HowBig2 * sizeof(float));
	/* make the key */
	if ((key2 = ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile2", 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid2 = shmget(key2, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	dataShared2 = (float *)shmat(shmid2, (void *)0, 0);

	if (dataShared2 == (float *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<HowBig2; i++)
	{
		dataShared2[i] = 0.0;
	}

	return dataShared2;
}