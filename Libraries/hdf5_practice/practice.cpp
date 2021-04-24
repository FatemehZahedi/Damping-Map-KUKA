#include "DataSet2D.h"
#include <unistd.h>

int main(){

    std::string filename = "myhdf.h5";
    H5File * file;
    try{
        /* Open file if it exists */
        file = new H5File(filename.c_str(), H5F_ACC_RDWR);
    }
    catch(const FileIException&){
        /* Create file if it does not exist */
        file = new H5File(filename.c_str(), H5F_ACC_TRUNC);
    }

    int nCols = 2;
    int buff[nCols] = {0, 0};


    std::string dsetNameBase = "test";
    int dsetCounter = 1;
    std::string dsetName = dsetNameBase;
    std::string dsetNameUnsuc = dsetName;
    if (file->exists(dsetNameUnsuc)){
        while (file->exists(dsetNameUnsuc)){
            dsetNameUnsuc = dsetNameBase + std::string("_unsuccessful_") + std::to_string(dsetCounter);
            dsetCounter++;
        }
        file->move(dsetName, dsetNameUnsuc);
    }
    DataSet2D<int> dset(file, dsetName, nCols);

    for (int i=0; i<15000; i++){
        /* fill buffer */
        for (int iCol=0; iCol<nCols; iCol++){
            buff[iCol] = i;
        }
        dset.AppendRow(buff);
        usleep(500);
    }
    dset.~DataSet2D();
    file->close();
}
