#include <udp_server.h>

#define _USE_MATH_DEFINES
#include <cmath>

using std::vector;

vector<double> linspace(double start, double stop, int n) {
    vector<double> array;
    double step = (stop-start)/(n-1);

    while(start <= stop) {
        array.push_back(start);
        start += step;           // could recode to better handle rounding errors
    }
    return array;
}


int main() {

    // Get user input for udp/ip address and port
	std::string addr;
	int port;

	std::cout << "Enter IP Address" << std::endl;
	std::cin >> addr;

	std::cout << "Enter Port" << std::endl;
	std::cin >> port;

	// Generate x,y lemniscate data
	double a = 2;
	double b = 2*sqrt(2);
	double t1 = 0;
	double t2 = 2*M_PI;
	double n = 3000;
	vector<double> t = linspace(t1, t2, n);
	vector<double> x = t;
	vector<double> y = t;
	for (int i=0; i<t.size(); i++){
		x[i] = a*cos(t[i])/(1+ pow(sin(t[i]), 2));
		y[i] = b*sin(t[i])*cos(t[i])/(1+ pow(sin(t[i]), 2));
	}

	// Create UDP Server and Connect to Client
	UDPServer server(addr, port);
	server.ConnectClient();

	// Send Data
	double data[2];
	int i = 0;
	while (1){
		data[0] = x[i];
		data[1] = y[i];
        server.ReconnectIfNecessary();
		server.Send(data, 2);
		i = (i+1)%((int) n);
		usleep(1000);
	}

	return 0;
}
