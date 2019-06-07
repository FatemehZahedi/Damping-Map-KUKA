#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class EmgClient{
private:
    bool _bRecordData = false;
    std::string _ipAddr;
    int         _port;
    float _data[10000][16];

    void Connect();
    void ReceiveForever();
    void ResetData();
public:
    EmgClient(std::string ipAddr, int port);
    void RecordStart();
    void RecordStop();
};


EmgClient::EmgClient(std::string ipAddr, int port){
    this->_ipAddr = ipAddr;
    this->_port = port;

}

void EmgClient::RecordStart(){
    this->_bRecordData = true;
}

void EmgClient::RecordStop(){
    this->_bRecordData = false;
}

void EmgClient::ReceiveForever(){
    int hello = 0;
}

void ResetData(){
    memset(&(this->_data[0][0]), 0, sizeof(this->data));
}

int main(int argc, char** argv){
    if (argc < 3){
        printf("2 required command line parameters\n");
        printf("\t1. IP Addr\n");
        printf("\t2. Port\n");
        return 0;
    }

    std::string ipAddr(argv[1]);
    int port = atoi(argv[2]);
    EmgClient emg(ipAddr, port);

    /* Put Elements Into Array */
    int nRows = 3;
    int nCols = 6;
    int arr[nRows][nCols];
    memset(arr, 0, sizeof(arr));
    
    int count = 0;
    int tempRow[nCols];
    int iRow = 0;
    int iCol = 0;
    for (iRow = 0; iRow<nRows; ++iRow){
        // zero out tempRow
        memset(tempRow, 0, nCols*sizeof(int));
        // fill tempRow
        for (iCol=0; iCol<nCols; iCol++){
            tempRow[iCol] = count;
            count++;
        }
        // copy tempRow to row of arr
        memcpy(&arr[iRow][0], tempRow, nCols*sizeof(int));
    }
    
    // print array
    for (iRow=0; iRow<nRows; iRow++){
        printf("\n");
        for (iCol=0; iCol<nCols; iCol++){
            printf("%d\t", arr[iRow][iCol]);
        }
    }


}
