#include "H5Cpp.h"
#include <string.h>

using namespace H5;

DataSet InitDataSet2D(H5Location * h5loc, std::string dsetName, hsize_t nCols, const DataType & data_type, hsize_t chunkRows = 10000){
    /* Init DataSpace */
    int rank = 2;
    hsize_t dims[rank] = {0, nCols};
    hsize_t dimsMax[rank] = {H5S_UNLIMITED, nCols};
    DataSpace dspace(rank, dims, dimsMax);

    /* Init Dataset Property List */
    DSetCreatPropList dsPropList;
    hsize_t chunkDims[rank] = {chunkRows, nCols};
    dsPropList.setChunk(rank, chunkDims);

    /* Create DataSet */
    DataSet dset = h5loc->createDataSet(dsetName, data_type, dspace, dsPropList);
    return dset;
}

template <typename T>
void AppendToDataSet2D(DataSet & dset, T * buff, hsize_t nRowsToAppend, const DataType & data_type){
    /* Get current dimensions, will be used to calculate offset */
    DataSpace fspace = dset.getSpace();
    int rank = 2;
    hsize_t dimsOld[rank];
    rank = fspace.getSimpleExtentDims(dimsOld);

    /* Calculate new dataspace dimensions */
    hsize_t offset[rank] = {dimsOld[0], 0};
    hsize_t dimsNew[rank] = {dimsOld[0]+nRowsToAppend, dimsOld[1]};
    hsize_t dimsWrite[rank] = {nRowsToAppend, dimsOld[1]};

    /* Extend dataset dimensions */
    dset.extend(dimsNew);

    /* Specify where in file space the new data will be written */
    fspace = dset.getSpace();
    fspace.selectHyperslab(H5S_SELECT_SET, dimsWrite, offset);

    /* Specify memory space */
    DataSpace mspace(rank, dimsWrite);
    mspace.selectAll();

    /* Write To DataSet */
    dset.write(buff, data_type, mspace, fspace);
}


int main(int argc, char** argv){

    /* create file */
    H5File file("myhdf.h5", H5F_ACC_EXCL);

    /* init data */
    int nPages  = 2;
    int nRows   = 10000;
    int nCols   = 2;
    int data[nPages][nRows][nCols];

    int temp[2] = {0, 1};
    for (int iRow = 0; iRow<nRows; iRow++){
        memcpy(&(data[0][iRow][0]), temp, sizeof(int)*nCols);
    }

    temp[0] = 2;
    temp[1] = 3;

    for (int iRow = 0; iRow<nRows; iRow++){
        memcpy(&(data[1][iRow][0]), temp, sizeof(int)*nCols);
    }

    DataSet dset = InitDataSet2D(&file, "Data", (hsize_t) nCols, PredType::NATIVE_INT);
    AppendToDataSet2D(dset, &(data[0][0][0]), (hsize_t) nRows-1000, PredType::NATIVE_INT);
    AppendToDataSet2D(dset, &(data[1][0][0]), (hsize_t) nRows, PredType::NATIVE_INT);

    file.close();
}
