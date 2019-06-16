#include <string>
#include "H5Cpp.h"

using namespace H5;


H5File * CreateOrOpenFile(const std::string & filename){
    Exception::dontPrint();
    H5File * file = 0;

    try{
        /* Open file if it exists */
        file = new H5File(filename.c_str(), H5F_ACC_RDWR);
    }
    catch(const FileIException&){
        /* Create file if it does not exist */
        file = new H5File(filename.c_str(), H5F_ACC_TRUNC);
    }

    return file;
}

Group * CreateOrOpenGroup(H5Location * h5loc, H5std_string & group_name){

    Exception::dontPrint();
    Group * group = 0;

    try{
        /* Open group if it exists */
        group = new Group(h5loc->openGroup(group_name));
    } catch(Exception & e){
        group = new Group(h5loc->createGroup(group_name));
    }

    return group;
}

// TrialDatasets CreateTrialDatasets(H5Location * h5loc){
//     TrialDatasets td;
//
//     // Define dataspace dimensions
//     hsize_t dims_ja[2] = {10000, 7};
//     hsize_t dims_force[2] = {10000, 2};
//     hsize_t dims_pf[2] = {10000, 1};
//     hsize_t dims_eepo[2] = {10000, 6};
//     hsize_t maxdims_ja[2] = {H5S_UNLIMITED, 7};
//     hsize_t maxdims_force[2] = {H5S_UNLIMITED, 2};
//     hsize_t maxdims_pf[2] = {H5S_UNLIMITED, 1};
//     hsize_t maxdims_eepo[2] = {H5S_UNLIMITED, 6};
//
//     // Define dataspaces
//     DataSpace dataspace_ja = DataSpace(2, dims_ja, maxdims_ja);
//     DataSpace dataspace_force = DataSpace(2, dims_force, maxdims_force);
//     DataSpace dataspace_pf = DataSpace(2, dims_pf, maxdims_pf);
//     DataSpace dataspace_eepo = DataSpace(2, dims_eepo, maxdims_eepo);
//
//     // Property List
//     DSetCreatPropList prop_ja;
//     DSetCreatPropList prop_force;
//     DSetCreatPropList prop_pf;
//     DSetCreatPropList prop_eepo;
//
//     prop_ja.setChunk(2, dims_ja);
//     prop_force.setChunk(2, dims_force);
//     prop_pf.setChunk(2, dims_pf);
//     prop_eepo.setChunk(2, dims_eepo);
//
//
//     // Create or Open Datasets
//     try{
//         td.JointAngle = h5loc->openDataSet("JointAngle");
//     }
//     catch{
//         td.JointAngle = h5loc->createDataSet("JointAngle",
//         PredType::NATIVE_DOUBLE,
//         dataspace_ja,
//         prop_ja);
//     }
//
//     try{
//         td.Force = h5loc->openDataSet("Force");
//     }
//     catch{
//         td.Force = h5loc->createDataSet("Force",
//         PredType::NATIVE_DOUBLE,
//         dataspace_force,
//         prop_force);
//     }
//
//     try{
//         td.PerturbFlag = h5loc->openDataSet("PerturbFlag");
//     }
//     catch{
//         td.PerturbFlag = h5loc->createDataSet("PerturbFlag",
//         PredType::NATIVE_INT,
//         dataspace_pf,
//         prop_pf);
//     }
//
//     try{
//         td.EndEffectorPO = h5loc->openDataSet("EndEffectorPO");
//     }
//     catch{
//         td.EndEffectorPO = h5loc->createDataSet("EndEffectorPO",
//         PredType::NATIVE_DOUBLE,
//         dataspace_eepo,
//         prop_eepo);
//     }
//
//     return td;
// }

DataSet InitDataSet2D(H5Location & h5loc, std::string dsetName, hsize_t nCols, const DataType & data_type, hsize_t chunkRows = 10000){
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
    DataSet dset = h5loc.createDataSet(dsetName, data_type, dspace, dsPropList);
    return dset;
}

template <typename T>
void AppendToDataSet2D(DataSet & dset, T * buff, hsize_t nRowsToAppend, int nCols, const DataType & data_type){
    /* copy buff in case value pointed to by buff changes */
    T temp[(int)nRows][nCols];
    memcpy(temp, buff, sizeof(T)*(int)nRows*nCols);

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
    dset.write(temp, data_type, mspace, fspace);
}

void CreateStringAttribute(H5Object * h5obj, H5std_string attr_name, H5std_string attr_val){
    if (!(h5obj->attrExists(attr_name))){
        // Create new dataspace for attribute -- H5S_SCALAR means the dataspace has 1 element
        DataSpace ds = DataSpace(H5S_SCALAR);

        // Create new string datatype for attribute
        StrType strdatatype(PredType::C_S1, 256); // of length 256 characters

        // Create attribute and write to it
        Attribute attr = h5obj->createAttribute(attr_name, strdatatype, ds);
        attr.write(strdatatype, attr_val);
    }
}

void CreateNumericAttribute(H5Object * h5obj, H5std_string attr_name, double attr_val){
    if (!(h5obj->attrExists(attr_name))){
        // Create new dataspace for attribute -- H5S_SCALAR means the dataspace has 1 element
        DataSpace ds = DataSpace(H5S_SCALAR);

        // Create float datatype
        FloatType floatdatatype(PredType::NATIVE_DOUBLE);

        // Create attribute and write to it
        Attribute attr = h5obj->createAttribute(attr_name, floatdatatype, ds);
        attr.write(floatdatatype, &attr_val);
    }
}
