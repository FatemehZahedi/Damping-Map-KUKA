#include "H5Cpp.h"
#include <string>
#include <string.h>
#include <exception>
#include <thread>
using namespace H5;

struct InvalidDataset2DType : public std::exception {
   const char * what () const throw () {
       std::string err_msg = "Invalid type given for DataSet2D<type>. Valid types:" +
                             std::string("\n\tint8_t") +
                             std::string("\n\tuint8_t") +
                             std::string("\n\tint16_t") +
                             std::string("\n\tuint16_t") +
                             std::string("\n\tint32_t") +
                             std::string("\n\tuint32_t") +
                             std::string("\n\tint64_t") +
                             std::string("\n\tuint64_t") +
                             std::string("\n\tfloat") +
                             std::string("\n\tdouble") +
                             std::string("\nor equivalent types.\n");
      return err_msg.c_str();
   }
};




template <typename T>
class DataSet2D{
private:
    /* 2D data array with 2 'layers' of data */
    bool _dataMemoryAllocated = false;
    T *** _data;
    const int _nLayers = 2;
    const int _nRows = 10000;
    int _nCols;

    /* Variables to keep track of what needs to be written to DataSet */
    int _currLayerRowsToAppend = 0;
    int _currLayer = 0;

    /* H5 Variables */
    int _chunkRows = 10000;
    DataType _h5DataType;
    DataSet _dataset;

public:
    DataSet2D(H5Location * h5loc, std::string dsetName, int nCols);
    ~DataSet2D();
    void FreeMemory();
    void AppendRow(T * buff);
    void WriteToDataSet(int layerIndex, int rowsToAppend);
    DataSet GetH5DataSet();
};


template <typename T>
DataSet2D<T>::DataSet2D(H5Location * h5loc, std::string dsetName, int nCols){
    _nCols = nCols;

    /* Allocate memory for _data */
    _data = new T**[_nLayers];
    for (int iLayer=0; iLayer<_nLayers; iLayer++){
        _data[iLayer] = new T*[_nRows];
        for (int iRow=0; iRow<_nRows; iRow++){
            _data[iLayer][iRow] = new T[_nCols];
        }
    }
    _dataMemoryAllocated = true;

    /* Get H5 DataType*/
    if (std::is_same<T, int8_t>::value){
        _h5DataType = PredType::NATIVE_INT8;
    }
    else if (std::is_same<T, uint8_t>::value){
        _h5DataType = PredType::NATIVE_UINT8;
    }
    else if (std::is_same<T, int16_t>::value){
        _h5DataType = PredType::NATIVE_INT16;
    }
    else if (std::is_same<T, uint16_t>::value){
        _h5DataType = PredType::NATIVE_UINT16;
    }
    else if (std::is_same<T, int32_t>::value){
        _h5DataType = PredType::NATIVE_INT32;
    }
    else if (std::is_same<T, uint32_t>::value){
        _h5DataType = PredType::NATIVE_UINT32;
    }
    else if (std::is_same<T, int64_t>::value){
        _h5DataType = PredType::NATIVE_INT64;
    }
    else if (std::is_same<T, uint64_t>::value){
        _h5DataType = PredType::NATIVE_UINT64;
    }
    else if (std::is_same<T, float>::value){
        _h5DataType = PredType::NATIVE_FLOAT;
    }
    else if (std::is_same<T, double>::value){
        _h5DataType = PredType::NATIVE_DOUBLE;
    }
    else{
        throw InvalidDataset2DType();
    }

    /* Init H5 DataSpace */
    int rank = 2;
    hsize_t dims[rank] = {0, (hsize_t) _nCols};
    hsize_t dimsMax[rank] = {H5S_UNLIMITED, (hsize_t) _nCols};
    DataSpace dspace(rank, dims, dimsMax);

    /* Init Dataset Property List */
    DSetCreatPropList dsPropList;
    hsize_t chunkDims[rank] = {(hsize_t) _chunkRows, (hsize_t) _nCols};
    dsPropList.setChunk(rank, chunkDims);

    /* Create DataSet */
    _dataset = h5loc->createDataSet(dsetName, _h5DataType, dspace, dsPropList);
}


template<typename T>
DataSet2D<T>::~DataSet2D(){
    if (_currLayerRowsToAppend < 0){
        WriteToDataSet(_currLayer, _currLayerRowsToAppend);
    }
    // FreeMemory();
}

template<typename T>
void DataSet2D<T>::FreeMemory(){
    /* Deallocate memory for _data */
    if (_dataMemoryAllocated){
        for (int iLayer=0; iLayer<_nLayers; iLayer++){
            for (int iRow=0; iRow<_nRows; iRow++){
                delete [] _data[iLayer][iRow];
            }
            delete [] _data[iLayer];
        }
        delete [] _data;
    }
}

template<typename T>
void DataSet2D<T>::AppendRow(T * buff){
    memcpy(&(_data[_currLayer][_currLayerRowsToAppend][0]), buff, sizeof(T)*_nCols);
    _currLayerRowsToAppend++;
    /* Write to DataSet if Layer is full */
    if (_currLayerRowsToAppend >= _nRows){
        std::thread t(&DataSet2D<T>::WriteToDataSet, this, this->_currLayer, this->_currLayerRowsToAppend);
        t.detach();
        /* Reset row count and current layer */
        _currLayerRowsToAppend = 0;
        if (_currLayer == 0){
            _currLayer = 1;
        }
        else{
            _currLayer = 0;
        }
    }
}

template<typename T>
void DataSet2D<T>::WriteToDataSet(int layerIndex, int rowsToAppend){
    /* Get current dimensions, will be used to calculate offset */
    DataSpace fspace = _dataset.getSpace();
    int rank = 2;
    hsize_t dimsOld[rank];
    rank = fspace.getSimpleExtentDims(dimsOld);

    /* Calculate new dataspace dimensions */
    hsize_t offset[rank] = {dimsOld[0], 0};
    hsize_t dimsNew[rank] = {dimsOld[0]+ (hsize_t) rowsToAppend, dimsOld[1]};
    hsize_t dimsWrite[rank] = {(hsize_t) rowsToAppend, dimsOld[1]};

    /* Extend dataset dimensions */
    _dataset.extend(dimsNew);

    /* Specify where in file space the new data will be written */
    fspace = _dataset.getSpace();
    fspace.selectHyperslab(H5S_SELECT_SET, dimsWrite, offset);

    /* Specify memory space */
    DataSpace mspace(rank, dimsWrite);
    mspace.selectAll();

    /* Write To DataSet */
    _dataset.write(&(_data[layerIndex][0][0]), _h5DataType, mspace, fspace);
}

template<typename T>
DataSet DataSet2D<T>::GetH5DataSet(){
    return _dataset;
}
