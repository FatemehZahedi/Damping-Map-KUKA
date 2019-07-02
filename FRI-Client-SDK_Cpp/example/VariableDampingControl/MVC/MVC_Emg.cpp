#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>
#include "UdpServer.h"
#include "TrignoEmgClient.h"
#include "H5FunctionsNMCHRL.h"
#include <algorithm>
/* Boost filesystem */
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
/* HDF5 */
#include "H5Cpp.h"


using namespace std;
using namespace boost::filesystem;



// Main
int main(int argc, char** argv)
{

	/* Command line arguments */
	std::string emgIpAddr;
	int subjectNumber;
	if (argc < 3){
		printf("Use: KukaVariableDamping <SubjectNumber> <EMG IP Addressr>\n");
		exit(1);
	}
	else{
		subjectNumber = atoi(argv[1]);
		emgIpAddr = std::string(argv[2]);
	}

	/* Check EMG use */
	TrignoEmgClient emgClient;
	if (useEmg){
		emgClient.SetIpAddress(emgIpAddr);
		emgClient.ConnectDataPort();
		emgClient.ConnectCommPort();
		if (emgClient.IsCommPortConnected()){
			emgClient.SendCommand(1); // this command signals the emg server to send readings to Data Port
			std::thread emgReceiveThread(&TrignoEmgClient::ReceiveDataStream, &emgClient);
			emgReceiveThread.detach();
		}
	}

	/* Make path */
	path p_base = current_path();
	std::string subjectDir = "Subject" + std::to_string(subjectNumber);
	path p_subject = path(p_base.string()) /= path(subjectDir);
	create_directory(p_subject);

	/* Make Emg path */
	std::string mcvFile = "MVC.txt";
	path p_mvc = path(p_subject.string()) /= path(mvcFile);


	sleep(10);
	printf("Starting EMG Write\n");
	emgClient.StartWritingFileStream(p_mvc);




	return 1;
}
