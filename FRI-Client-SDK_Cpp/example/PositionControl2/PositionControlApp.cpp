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

// UDP Server Headers
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;

#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"  //ip address set for KONI connection
#define SHM_SIZE 1024

FILE *NewDataFile(void);


int *MakeFloatSharedMemory(int HowBig);
float *MakeFloatSharedMemory2(int HowBig2);


// UDP-----------------------------------------------------------------------------------

class UDPServer {
private:
	// UDP/IP socket info
	std::string _addr;
	int _port;
	int _sockfd;

	// Server/Client address info
	struct sockaddr_in _servaddr;
	struct sockaddr_in _cliaddr;
	socklen_t _cliaddrlen = sizeof(_cliaddr);

	// Recv/send limit
	const int _maxlen = 1024;

	// Connection State
	bool _connected = false;

	// select(2) related data
	fd_set _readset;
	struct timeval _select_timeout = { .tv_sec = 0,.tv_usec = 0 };

public:
	// Class Methods
	UDPServer(std::string addr, int port);
	std::string GetAddress();
	int GetPort();
	bool IsConnected() const;
	void ConnectClient();
	void ConnectIfNecessary();
	template<typename T>
	void Send(T* data, int ndata);
};

UDPServer::UDPServer(std::string addr, int port) : _port(port), _addr(addr) {

	// Initialize socket
	_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (_sockfd < 0) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}

	// Reset and fill server address structs
	memset(&_servaddr, 0, sizeof(_servaddr));
	memset(&_cliaddr, 0, sizeof(_cliaddr));

	_servaddr.sin_family = AF_INET;
	inet_pton(AF_INET, _addr.c_str(), &_servaddr.sin_addr);
	_servaddr.sin_port = htons(_port);

	// Bind the socket with the server address
	if (bind(_sockfd, (const struct sockaddr *)&_servaddr, sizeof(_servaddr)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
}

void UDPServer::ConnectIfNecessary() {
	/*
	select(2) system call monitors if there is an incoming messge from the client
	If there is, select(2) will return a number greater than 0.  We will then connect
	run ConnectClient() which reads in the message and gets the client's ip info
	so the server can send messages to it
	*/
	FD_ZERO(&_readset);
	FD_SET(_sockfd, &_readset);
	int ret = select(_sockfd + 1, &_readset, NULL, NULL, &_select_timeout);
	if (ret > 0) {
		ConnectClient();
	}
}

std::string UDPServer::GetAddress() {
	// return server's ip address
	return _addr;
}

int UDPServer::GetPort() {
	// return server's port
	return _port;
}

bool UDPServer::IsConnected() const {
	// return server-client connection status
	return _connected;
}

void UDPServer::ConnectClient() {
	/*
	recvfrom() system call reads the message into the buffer and fills the _cliaddr
	struct with the client's ip info.  The message in the buffer is not important.
	We only do this so we get the client's ip address info
	*/
	char buffer[_maxlen];
	int n;
	n = recvfrom(_sockfd, (char *)buffer, _maxlen,
		MSG_WAITALL, (struct sockaddr *) &_cliaddr, &_cliaddrlen);
	_connected = true;
}

template<typename T>
void UDPServer::Send(T* data, int ndata) {
	// Connect/Reconnect to client if necessary (get's clients ip address info)
	ConnectIfNecessary();
	// Send data if the server is connected to a client
	if (_connected) {
		sendto(_sockfd, (const T *)data, sizeof(data)*ndata,
			MSG_DONTWAIT, (const struct sockaddr *) &_cliaddr, _cliaddrlen);
	}
}






//------------------------------------------------------------------------------------------------------------

// Main
int main(int argc, char** argv)
{


	// UDP Server address and port hardcoded now -- change later
	std::string udp_addr("192.168.0.103");
	int udp_port = 50000;
	UDPServer udp_server(udp_addr, udp_port);



	//******************---------------------------------------
	//---------------------------------------
	/*double mag = 0.025;
	double dur = 150;*/
	double mag = 0.05;
	double dur = 300;
	double ftx;
	double fty;
	double ftx_un;
	double fty_un;
	double zerox = 0;
	double zeroy = 0;
	double x_disp;
	double y_disp;
	double ftx_0 = 0.0;
	double fty_0 = 0.0;
	int *data;	//pointer to shared memory
	data = MakeFloatSharedMemory(2);
	float *data2; //pointer to shared memory
	data2 = MakeFloatSharedMemory2(2);
	int cc = 1;
	double al = 0.5;
	int firstIt = 0;
	int trigger = 1;
	int w = 0;
	double random_num;
	double ft[2];
	int steady = 0;
	int steady2 = 0;
	int steady3 = 0;
	int steady4 = 0;
	int perturb_flag = 0;
	int rr = 1;

	double phi_euler = 0;
	double theta_euler = 0;
	double psi_euler = 0;

	int flag_finish = 0;
	int flag_ex = 0;
	int flag_I = 1;
	int flag_p = 0;
	int flag_p2 = 0;
	int fi1 = 0;
	int fi2 = 0;
	int fi3 = 0;
	int fs = 0;
	int mode = 1;
	/*int fj1 = 0;
	int fj2 = 0;*/

	double xy_coord[16];
	memset(xy_coord, 0, sizeof(double) * 16);

	//-----------------------------------------------
	//********************-----------------------------------------------



	// DH Parameter----------------------------------------------------
	MatrixXd alpha(1, 7); alpha << M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, 0;
	MatrixXd a(1, 7); a << 0, 0, 0, 0, 0, 0, 0;
	MatrixXd d(1, 7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
	MatrixXd theta(1, 7); theta << 0, 0, 0, 0, 0, 0, 0;



	//******************---------------------------------------
	//---------------------------------------
	// May be changed-------------
	MatrixXd x_0(6, 1); x_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_test(6, 1); x_test << 0, 0, 0, 0, 0, 0;
	MatrixXd x_00(6, 1); x_00 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_incr(6, 1); x_incr << 0, 0, 0, 0, 0, 0;
	MatrixXd x_incr_0(6, 1); x_incr_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd test_new(6, 1); test_new << 0, 0, 0, 0, 0, 0;


	MatrixXd qc(6, 1); qc << 0, 0, 0, 0, 0, 0;
	MatrixXd delta_q(6, 1); delta_q << 0, 0, 0, 0, 0, 0;
	MatrixXd q_freeze(6, 1); q_freeze << 0, 0, 0, 0, 0, 0;

	MatrixXd q_0(6, 1); q_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd t_0(6, 1); t_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_e(6, 1); x_e << 0, 0, 0, 0, 0, 0;
	MatrixXd force(6, 1); force << 0, 0, 0, 0, 0, 0;
	MatrixXd q_new(6, 1); q_new << 0, 0, 0, 0, 0, 0;
	MatrixXd q_delay(6, 1); q_delay << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new(6, 1); x_new << 0, 0, 0, 0, 0, 0;
	MatrixXd v(6, 1); v << 0, 0, 0, 0, 0, 0;
	MatrixXd acc(6, 1); acc << 0, 0, 0, 0, 0, 0;
	MatrixXd Fnew(6, 1); Fnew << 0, 0, 0, 0, 0, 0;
	MatrixXd q_old(6, 1); q_old << 0, 0, 0, 0, 0, 0;

	/////////////////
	MatrixXd x_new2(6, 1); x_new2 << 0, 0, 0, 0, 0, 0;
	MatrixXd Fnew2(6, 1); Fnew2 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_02(6, 1); x_02 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_002(6, 1); x_002 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_03(6, 1); x_03 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_003(6, 1); x_003 << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new3(6, 1); x_new3 << 0, 0, 0, 0, 0, 0;
	/////////////////

	//-------------------------------------------------
	//*****************************--------------------------
	//points and values-------------------------------------

	double radius_e = 0.005;
	double rangex_ = -0.18;
	double rangex = 0.18;
	double rangey_ = 0.58;
	double rangey = 0.94;
	double d_r = (rangex * 2) / 20;
	double u_r = d_r - radius_e;
	double ex_r = d_r / 4;

	int t_r;
	MatrixXd time_ran(7, 1); time_ran << 0.7, 1, 0.9, 1.2, 0.8, 1.4, 0.5;

	int chosen;
	int n_v = 0;
	int i_c;
	int j_c;
	int chosen_damping;
	int chosen_point;
	int flag_chosen = 1;
	int flag_v = 0;
	int random_v;
	MatrixXd desired(2, 1); desired << 0, 0;
	MatrixXd point(2, 1); point << 0, 0;
	MatrixXd P_ex(2, 1); P_ex << 0, 0;
	MatrixXd Vector(30, 1); Vector << MatrixXd::Zero(30, 1);
	MatrixXd damping_values(5, 6); damping_values << 10, 0, -10, -20, -25, -30,
		10, 0, -10, -20, -25, -30,
		10, 0, -10, -20, -25, -30,
		10, 0, -10, -20, -25, -30,
		10, 0, -10, -20, -25, -30;
	/*MatrixXd damping_values(5, 6); damping_values << 5, 0, -5, -10, -15, -20,
	5, 0, -5, -10, -15, -20,
	5, 0, -5, -10, -15, -20,
	5, 0, -5, -10, -15, -20,
	5, 0, -5, -10, -15, -20;*/

	MatrixXd random1(30, 1); random1 << 1, 2, 3, 16, 11, 30, 7, 28, 17, 14, 8, 5, 29, 21, 25, 27, 26, 19, 15, 22, 23, 6, 4, 18, 24, 13, 9, 20, 10, 12;
	MatrixXd random2(30, 1); random2 << 2, 10, 4, 5, 25, 24, 15, 30, 21, 3, 8, 28, 12, 11, 17, 16, 26, 29, 18, 23, 22, 7, 1, 19, 20, 13, 14, 6, 9, 27;
	MatrixXd random3(30, 1); random3 << 8, 9, 17, 14, 18, 12, 2, 10, 26, 16, 21, 29, 20, 3, 7, 24, 30, 23, 19, 4, 1, 28, 27, 13, 22, 11, 5, 25, 15, 6;

	// Matrix for direction of perturbation
	MatrixXd random_mat(5, 6); random_mat << MatrixXd::Zero(5, 6);
	MatrixXd random_num1(5, 6); random_num1 << 2, 1, 2, 1, 1, 2,
											  1, 2, 2, 2, 2, 1,
											  2, 2, 1, 2, 2, 2,
											  2, 2, 2, 2, 2, 1,
											  1, 1, 1, 2, 2, 2;

	MatrixXd random_num2(5, 6); random_num2 << 1, 2, 1, 1, 2, 2,
											  1, 1, 2, 2, 2, 1,
											  1, 2, 2, 2, 1, 2,
											  1, 1, 1, 2, 1, 1,
											  2, 1, 1, 1, 1, 2;

	MatrixXd random_num3(5, 6); random_num3 << 1, 2, 1, 1, 2, 2,
											  2, 1, 2, 1, 2, 2,
											  2, 1, 1, 2, 2, 1,
											  2, 1, 2, 1, 2, 2,
											  2, 2, 1, 1, 1, 1;

	MatrixXd random_num4(5, 6); random_num4 << 1, 2, 2, 1, 1, 2,
											  2, 1, 1, 2, 1, 2,
											  2, 1, 2, 2, 2, 1,
											  2, 1, 1, 2, 1, 1,
											  1, 1, 2, 1, 2, 1;

	MatrixXd random_num5(5, 6); random_num5 << 2, 1, 2, 2, 2, 1,
											  1, 1, 1, 2, 1, 1,
											  2, 1, 1, 2, 2, 1,
											  2, 2, 1, 1, 1, 1,
											  2, 1, 1, 2, 1, 1;

	MatrixXd random_num6(5, 6); random_num6 << 1, 2, 1, 2, 2, 1,
											  2, 1, 1, 1, 1, 2,
											  1, 1, 2, 1, 1, 1,
											  1, 1, 1, 1, 1, 2,
											  2, 2, 2, 1, 1, 1;
	MatrixXd random_num7(5, 6); random_num7 << 2, 1, 2, 2, 1, 1,
											  2, 2, 1, 1, 1, 2,
											  2, 1, 1, 1, 2, 1,
											  2, 2, 2, 1, 2, 2,
											  1, 2, 2, 2, 2, 1;

	MatrixXd random_num8(5, 6); random_num8 << 2, 1, 2, 2, 1, 1,
											  1, 2, 1, 2, 1, 1,
											  1, 2, 2, 1, 1, 2,
											  1, 2, 1, 2, 1, 1,
											  1, 1, 2, 2, 2, 2;

	MatrixXd random_num9(5, 6); random_num9 << 2, 1, 1, 2, 2, 1,
											  1, 2, 2, 1, 2, 1,
											  1, 2, 1, 1, 1, 2,
											  1, 2, 2, 2, 2, 2,
											  2, 2, 1, 2, 1, 2;

	MatrixXd random_num10(5, 6); random_num10 << 1, 2, 1, 1, 1, 2,
											     2, 2, 2, 1, 2, 2,
											     1, 2, 2, 1, 1, 2,
											     1, 1, 2, 2, 2, 2,
											     1, 2, 2, 1, 2, 2;


	//----------------------------------------------------------
	// Environmental values
	// Initializing Stiffness Damping and Inertia

	MatrixXd stiffness(6, 6); stiffness << 0, 0, 0, 0, 0, 0, //toward varun desk
		0, 10000000, 0, 0, 0, 0, //up
		0, 0, 0, 0, 0, 0, //out toward workshop
		0, 0, 0, 1000000, 0, 0,
		0, 0, 0, 0, 1000000, 0,
		0, 0, 0, 0, 0, 1000000;

	MatrixXd damping(6, 6); damping << 30, 0, 0, 0, 0, 0,
		0, 100, 0, 0, 0, 0,
		0, 0, 30, 0, 0, 0,
		0, 0, 0, 0.5, 0, 0,
		0, 0, 0, 0, 0.5, 0,
		0, 0, 0, 0, 0, 0.5;

	MatrixXd inertia(6, 6); inertia << 10, 0, 0, 0, 0, 0,
		0, 0.000001, 0, 0, 0, 0,
		0, 0, 10, 0, 0, 0,
		0, 0, 0, 0.0001, 0, 0,
		0, 0, 0, 0, 0.0001, 0,
		0, 0, 0, 0, 0, 0.0001;



	//******************---------------------------------
	int sample = 200;
	MatrixXd tdata(6, sample); tdata.setZero(6, sample);
	//***********************-----------------------------


	//*********************-----------------------------------
	// First angles of Kuka
	MatrixXd qdata(6, sample); qdata.setZero(6, sample);//Initializing joint angles

														//************************----------------------------
														// May be changed
	MatrixXd t_sum(6, 1); t_sum.setZero(6, 1);
	MatrixXd q_sum(6, 1); q_sum.setZero(6, 1);
	MatrixXd t_ave(6, 1); t_ave << 0, 0, 0, 0, 0, 0;
	MatrixXd q_ave(6, 1); q_ave << 0, 0, 0, 0, 0, 0;
	MatrixXd torques_0(6, 1); torques_0 << 0, 0, 0, 0, 0, 0;
	MatrixXd t_e(6, 1); t_e << 0, 0, 0, 0, 0, 0;
	//***********************-----------------------------

	//**********************-----------------------------
	// Not sure what it is
	struct timespec start2, finish2;
	struct timeval start, end;
	long mtime, seconds, useconds;
	gettimeofday(&start, NULL);
	//*********************------------------------------

	// parse command line arguments
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

	FILE *OutputFile;
	FILE *fp;

	bool success = true;
	bool success2 = true;

	int count = 0;
	int enough = 0;
	int i = 0;


	int stime = 0;
	float sampletime = 0; //will be determined from FRI
	double MJoint[7] = { 0 };//{-1.776415,1.025433,-0.064599,1.656986,0.290444,-0.971846,-0.223775}; //last measured joint position (not sure time when controller measured, be careful using this for feedback loop)
	double ETorque[7] = { 0 }; //External torque :supposedly the torque applied to the robot (subtracts the torque due to the robot, ackording to Kuka)

	double MaxRadPerSec[7] = { 1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159 }; //absolute max velocity (no load from KUKA manual for iiwa 800)
																					 //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
	double MaxRadPerStep[7] = { 0 };//will be calculated
	double MaxJointLimitRad[7] = { 2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543 };//Max joint limits in radians (can be changed to further restrict motion of robot)
	double MinJointLimitRad[7] = { -2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543 }; //Min joint limits in radians (can be changed to further restrict motion of robot)
	double FirstPositionDelta[7] = { 0.0175,0.0175,0.0175,0.0175,0.0175,0.0175,0.0175 }; //maximum deviation from initial position in trajectory from start position in robot(radians)


																						 //Get value for the time step of the command cycle (used for determining max velocity)
																						 // sampletime=client.GetTimeStep();
	sampletime = 0.001;
	fprintf(stdout, "Sample Time:%f seconds\n", sampletime);

	//calculate max step value
	for (i = 0; i<7; i++)
	{
		MaxRadPerStep[i] = sampletime * MaxRadPerSec[i]; //converts angular velocity to discrete time step
	}


	// create new joint position client
	PositionControlClient client;
	client.intvalues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);




	// create new udp connection
	UdpConnection connection;


	// pass connection and client to a new FRI client application
	ClientApplication app(connection, client);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);




	//create file for output
	OutputFile = NewDataFile();


	client.NextJoint[0] = -1.5708;
	client.NextJoint[1] = 1.5708;
	client.NextJoint[2] = 0;
	client.NextJoint[3] = 1.5708;
	client.NextJoint[4] = 0;
	client.NextJoint[5] = -1.5708;
	client.NextJoint[6] = -0.958709;


	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));

	//*******************************---------------------------------------
	// I don't understand them
	while (!enough)
	{
		clock_gettime(CLOCK_MONOTONIC, &start2);
		//---------------------------timestamp----------------------------------
		gettimeofday(&end, NULL);
		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		//----------------------------------------------------------------------


		success = app.step();//step through program

		if (client.KukaState == 4)
		{
			count++; //count initialized at 0
			if (count == 1)//first time inside
			{
				sampletime = client.GetTimeStep();
				//calculate max step value
				for (i = 0; i < 7; i++)
				{
					client.MaxRadPerStep[i] = sampletime * MaxRadPerSec[i]; //converts angular velocity to discrete time step
				}

			}
			//*********************-------------------------------------------------------------


			memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7); //gets the most recent measured joint value
			memcpy(ETorque, client.GetExtTor(), sizeof(double) * 7);//gets the external torques at the robot joints (supposedly subtracts the torques caused by the robot)

																	// Forward Kinematic
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
			MatrixXd T02(4, 4); T02 << T01 * A2;
			MatrixXd T03(4, 4); T03 << T02 * A3;
			MatrixXd T04(4, 4); T04 << T03 * A4;
			MatrixXd T05(4, 4); T05 << T04 * A5;
			MatrixXd T06(4, 4); T06 << T05 * A6;


			//*******************--------------------------

			point(0) = T06(0, 3);
			x_disp = T06(0, 3);
			data2[0] = x_disp;
			point(1) = T06(2, 3);
			y_disp = T06(2, 3);
			data2[1] = y_disp;

			//*******************----------------------------

			// Inverse Kinematic

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

			//***********

			MatrixXd Tphi(6, 6); Tphi << 1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, -sin(phi_euler), cos(phi_euler)*sin(theta_euler),
				0, 0, 0, 0, cos(phi_euler), sin(phi_euler)*sin(theta_euler),
				0, 0, 0, 1, 0, cos(theta_euler);

			MatrixXd Ja(6, 6); Ja << Tphi.inverse()*Jg;

			//*****************


			//**************************--------------------
			if (firstIt == 0)//first time inside
			{
				firstIt = 1;
				std::cout << firstIt << std::endl;

				x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
				x_03 << x_e;
				x_003 << x_e;
				x_new << x_e;
			}
			//**********************------------------------

			//*********************-------------------------
			//??
			ftx = (double)data[0] / 1000000 - zerox; //toward varun desk
			ftx_un = (double)data[0] / 1000000 - zerox;

			fty = (double)data[1] / 1000000 - zeroy;
			fty_un = (double)data[1] / 1000000 - zeroy;

			ftx = al * ftx + (1 - al)*ftx_0;
			ftx_0 = ftx;
			fty = al * fty + (1 - al)*fty_0;
			fty_0 = fty;

			force << ftx, 0, fty, 0, 0, 0;

			steady = steady + 1;

			perturb_flag = 0;

			//**********************---------------------------

			if (flag_I == 1)
			{
				damping(0, 0) = 30;
				flag_I = 0;
			}

			if (flag_v == 1)
			{
				flag_v = 0;
				flag_finish = 1;
				q_freeze << q_new;
			}
			/*if (flag_v == 1)
			{
			random_v = rand() % 3 + 1;
			flag_v = 0;
			n_v = 0;
			if (random_v == 3)
			{
			Vector << random3;
			}
			else if (random_v == 2)
			{
			Vector << random2;
			}
			else
			{
			Vector << random1;
			}
			}*/
			Vector << random1;
			random_mat << random_num1;

			if (flag_chosen == 1)
			{
				chosen = Vector(n_v);
				if (chosen % 6 == 0)
				{
					i_c = chosen / 6 - 1;
					j_c = chosen % 6 + 5;
				}
				else
				{
					i_c = chosen / 6;
					j_c = chosen % 6 - 1;
				}

				chosen_damping = damping_values(i_c, j_c);
				chosen_point = i_c;

				std::cout << "Point" << std::endl;
				std::cout << chosen_point << std::endl;
				std::cout << "B" << std::endl;
				std::cout << chosen_damping << std::endl;

				flag_chosen = 0;
			}
			//********************-----------------------------

			// Defining each point

			if (chosen_point == 0)
			{
				desired(0) = 0;
				desired(1) = 0.76;
			}
			else if (chosen_point == 1)
			{
				desired(0) = 0.06;
				desired(1) = 0.82;
			}
			else if (chosen_point == 2)
			{
				desired(0) = -0.06;
				desired(1) = 0.82;
			}
			else if (chosen_point == 3)
			{
				desired(0) = 0.06;
				desired(1) = 0.7;
			}
			else if (chosen_point == 4)
			{
				desired(0) = -0.06;
				desired(1) = 0.7;
			}


			//-----------------------------------------------------------

			if (steady > 2000 && -radius_e <= point(0) - desired(0) && point(0) - desired(0) <= radius_e)
			{
				if (-radius_e <= point(1) - desired(1) && point(1) - desired(1) <= radius_e)
				{
					steady4++;
					if (steady4 > 1000)
					{
						damping(0, 0) = chosen_damping;
						flag_ex = 1;
						//flag_p = 0;
						/*std::cout << "damping" << std::endl;
						std::cout << damping(0,0) << std::endl;*/

					}

				}
			}
			/*std::cout << "position" << std::endl;
			std::cout << point << std::endl;*/

			if (steady < 2000)
			{
				force << 0, 0, 0, 0, 0, 0;

				zerox = al * (double)data[0] / 1000000 + (1 - al)*zerox;
				zeroy = al * (double)data[1] / 1000000 + (1 - al)*zeroy;

				q_freeze << q_new;

			}
			// Step 2--------------------
			//
			/*if (steady > 6000)
			{
			damping(0, 0) = 0;
			damping(2, 2) = 0;
			}*/
			//
			//----------------------------
			/*std::cout << "mode" << std::endl;
			std::cout << mode << std::endl;*/

			if (flag_ex == 1 && flag_p == 0 && fi1 == 0)
			{
				mode = 2;
				P_ex(0) = desired(0) + 0.05;
				P_ex(1) = desired(1);
			}
			else if (flag_p == 1 && fi2 == 0)
			{
				mode = 2;
				P_ex(0) = desired(0) - 0.05;
				P_ex(1) = desired(1);
			}
			else
			{
				mode = 1;
				P_ex(0) = desired(0);
				P_ex(1) = desired(1);
			}

			// Step 3 -----------------------------------------------
			if (flag_ex == 1)
			{
				if (fi1 == 0 && point(0) - desired(0) >= 0.05) // Should be changed to x_new(0) == 0.05 && x_new(2)==1; the same for others too
				{
					fi1 = 1;
					flag_p++;
					fs = 1;
				}
				if (fs == 1 && fi2 == 0 && point(0) - desired(0) <= -0.05)
				{
					fi2 = 1;
					flag_p++;
					fs = 0;
				}
				//
				/*if (fj1 == 0 && x_new(2) >= 1.05)
				{
				fj1 = 1;
				flag_p++;
				}
				if (fj2 == 0 && x_new(2) <= 0.95)
				{
				fj2 = 1;
				flag_p++;
				}*/
				//

				if (fi3 == 0 && flag_p == 2 && -radius_e <= point(0) - desired(0) && point(0) - desired(0) <= radius_e)
				{
					fi3 = 1;
					flag_p++;
				}
			}


			if (flag_p < 3)
			{
				random_num = rand_mat(i_c, j_c);
				//random_num = rand() % 2 + 1;
				t_r = rand() % 6 + 0;
			}

			if (flag_p == 3)
			{
				steady2++;
				flag_ex = 0;
			}



			if (flag_p == 3 && steady2 >= time_ran(t_r) * 1000 && cc == 1 && steady > 2000)
			{
				std::cout << "flag_p" << std::endl;
				std::cout << flag_p << std::endl;
				std::cout << "random_num" << std::endl;
				std::cout << random_num << std::endl;

				if (trigger == 1)
				{
					trigger = 0;
					q_freeze << q_new;
					delta_q << 0, 0, 0, 0, 0, 0;
					x_incr_0 << 0, 0, 0, 0, 0, 0;
				}

				if (random_num == 1)
				{
					if (rr == 1)
					{
						x_incr << 0, 0, 0, 0, 0, 0;
						if (w == 1)
						{
							rr = 2;
							w = 1;
						}
					}

					if (rr == 2)
					{
						x_incr << -mag * (10.0*pow(w / dur, 3.0) - 15.0*pow(w / dur, 4.0) + 6.0*pow(w / dur, 5.0)), 0, 0, 0, 0, 0;
						perturb_flag = 7;
						if (w == dur)
						{
							cc = 0;
							x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
							x_03 << x_e;
							x_003 << x_e;
							x_new << x_e;
							rr = 1;
						}
					}

					qc << Ja.inverse()*(x_incr - x_incr_0);
					delta_q << delta_q + qc;
					q_new << q_freeze + delta_q;
					x_incr_0 << x_incr;

				}
				// next number----------------------------------
				if (random_num == 2)
				{
					if (rr == 1)
					{
						x_incr << 0, 0, 0, 0, 0, 0;
						if (w == 1)
						{
							rr = 2;
							w = 1;
						}
					}

					if (rr == 2)
					{
						x_incr << mag * (10.0*pow(w / dur, 3.0) - 15.0*pow(w / dur, 4.0) + 6.0*pow(w / dur, 5.0)), 0, 0, 0, 0, 0;
						perturb_flag = 7;
						if (w == dur)
						{
							cc = 0;
							x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
							x_03 << x_e;
							x_003 << x_e;
							x_new << x_e;
							rr = 1;
						}
					}

					qc << Ja.inverse()*(x_incr - x_incr_0);
					delta_q << delta_q + qc;
					q_new << q_freeze + delta_q;
					x_incr_0 << x_incr;

				}
				/*
				// next number----------------------------------
				if (random_num == 3)
				{
				if (rr == 1)
				{
				x_incr << 0, 0, 0, 0, 0, 0;
				if (w == 1)
				{
				rr = 2;
				w = 1;
				}
				}
				if (rr == 2)
				{
				x_incr << 0, 0, mag*(10.0*pow(w / dur, 3.0) - 15.0*pow(w / dur, 4.0) + 6.0*pow(w / dur, 5.0)), 0, 0, 0;
				perturb_flag = 7;
				if (w == dur)
				{
				cc = 0;
				x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
				x_03 << x_e;
				x_003 << x_e;
				x_new << x_e;
				}
				}
				qc << Ja.inverse()*(x_incr - x_incr_0);
				delta_q << delta_q + qc;
				q_new << q_freeze + delta_q;
				x_incr_0 << x_incr;
				}
				// next number----------------------------------
				if (random_num == 4)
				{
				if (rr == 1)
				{
				x_incr << 0, 0, 0, 0, 0, 0;
				if (w == 1)
				{
				rr = 2;
				w = 1;
				}
				}
				if (rr == 2)
				{
				x_incr << 0, 0, -mag * (10.0*pow(w / dur, 3.0) - 15.0*pow(w / dur, 4.0) + 6.0*pow(w / dur, 5.0)), 0, 0, 0;
				perturb_flag = 7;
				if (w == dur)
				{
				cc = 0;
				x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
				x_03 << x_e;
				x_003 << x_e;
				x_new << x_e;
				}
				}
				qc << Ja.inverse()*(x_incr - x_incr_0);//+(q_old-q_0)*0.3;
				delta_q << delta_q + qc;
				q_new << q_freeze + delta_q;
				x_incr_0 << x_incr;
				}*/

				w = w + 1;
			}
			//--------------------------------------------------------------

			stime = client.GetTimeStamp();

			// Step 5-----------------------------------------

			if (flag_p2 == 1)
			{
				steady3++;
			}
			if (steady3 > 4000)
			{
				flag_I = 1;
				flag_chosen = 1;
				n_v++;
				steady3 = 0;
				flag_p2 = 0;
				flag_p = 0;
				fi1 = 0;
				fi2 = 0;
				fi3 = 0;
				cc = 1;
				steady2 = 0;
				steady4 = 0;
			}

			if (n_v == 30)
			{
				n_v = 0;
				flag_v = 1;
			}
			//----------------------------------------------


			fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %1f %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d\n", count, MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6], force(0), force(2), perturb_flag, x_new(0), x_new(1), x_new(2), x_new(3), x_new(4), x_new(5), damping(0, 0), desired(0), desired(1), trigger, chosen, random_num);

			// Use commanded x--------------------------------------------------------------------------
			x_003 << x_03;
			x_03 << x_new;
			x_new << (inertia / (0.000001) + damping / (0.001) + stiffness).inverse()*(force + (inertia / (0.000001))*(x_03 - x_003) + stiffness * (x_e - x_03)) + x_03;

			//------------------------------------------------------------------------------------------

			if (steady > 2000)
			{
				if (x_new(2) >= 0.94)
				{
					x_new(2) = 0.94;
				}

				if (x_new(2) <= 0.58)
				{
					x_new(2) = 0.58;
				}

				if (x_new(0) >= 0.18)
				{
					x_new(0) = 0.18;
				}

				if (x_new(0) <= -0.18)
				{
					x_new(0) = -0.18;
				}
			}

			if (trigger == 1)
			{
				if (0.58 <= x_new(2) && x_new(2) <= 0.94)
				{
					if (-0.18 <= x_new(0) && x_new(0) <= 0.18)
					{
						qc << Ja.inverse()*(x_new - x_03);
						delta_q << delta_q + qc;
						q_new << delta_q + q_freeze;
					}
				}
			}

			if (steady < 2000)
			{
				q_new(0) = -1.5708;
				q_new(1) = 1.5708;
				q_new(2) = 0;
				q_new(3) = 1.5708;
				q_new(4) = 0;
				q_new(5) = -1.5708;
			}

			if (flag_finish == 1)
			{
				flag_v = 0;
				q_new << q_freeze;
			}


			client.NextJoint[0] = q_new(0);
			client.NextJoint[1] = q_new(1);
			client.NextJoint[2] = q_new(2);
			client.NextJoint[3] = q_new(3);
			client.NextJoint[4] = q_new(4);
			client.NextJoint[5] = q_new(5);
			client.NextJoint[6] = -0.958709;

			// Step 4-------------------------------------
			if (w == dur + 1)
			{
				trigger = 1;
				q_freeze << q_new;
				delta_q << 0, 0, 0, 0, 0, 0;
				x_new << x_e;
				w = 0;
				flag_p2 = 1;
			}
			//---------------------------------------------
			/*client.NextJoint[0] = -1.5708;
			client.NextJoint[1] = 1.5708;
			client.NextJoint[2] = 0;
			client.NextJoint[3] = 1.5708;
			client.NextJoint[4] = 0;
			client.NextJoint[5] = -1.5708;
			client.NextJoint[6] = -0.958709;*/

			// Send data to visualizer gui
			xy_coord[0] = mode;
			xy_coord[1] = desired(0);
			xy_coord[2] = desired(1);
			xy_coord[3] = d_r;
			xy_coord[4] = point(0);
			xy_coord[5] = point(1);
			xy_coord[6] = u_r;
			xy_coord[7] = P_ex(0);
			xy_coord[8] = P_ex(1);
			xy_coord[9] = ex_r;
			xy_coord[10] = damping(0, 0);
			udp_server.Send(xy_coord, 16);


		}
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