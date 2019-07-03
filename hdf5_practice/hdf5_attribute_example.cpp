#include <string>
#include <iostream>

#include "H5Cpp.h"

using std::cout;
using std::endl;

using namespace H5;

const H5std_string FILE_NAME("string_test.h5");
const H5std_string DS_NAME("Data Set 1");
const H5std_string ATTR_NAME("String Attribute");

int main(void) {

    // Create the named file
    H5File file = H5File(FILE_NAME , H5F_ACC_TRUNC);

    // Create new dataspace for the dataset
    const int rank = 3;
    const int dim1 = 2;
    const int dim2 = 2;
    const int dim3 = 2;
    hsize_t  dims[rank] = {dim1, dim2, dim3};
    DataSpace dataspace = DataSpace(rank, dims);

    // Create new datatype for the dataset
    FloatType datatype(PredType::NATIVE_FLOAT);

    // Create the dataset
    DataSet dataset = file.createDataSet(DS_NAME, datatype, dataspace);

    // Set up write buffer 'matrix'
    int q,r,s;
    float value;
    float matrix[dim1][dim2][dim3];
    for (q = 0; q < dim1; q++)
	for (r = 0; r < dim2; r++)
	    for (s = 0; s < dim3; s++)
	    {
		value = 1.111 + (q * r * s);
		matrix[q][r][s] = value;
	    }

    // Write data to the dataset
    dataset.write(matrix, datatype);

    // Create new dataspace for attribute
    DataSpace attr_dataspace = DataSpace(H5S_SCALAR);

    // Create new string datatype for attribute
    StrType strdatatype(PredType::C_S1, 256); // of length 256 characters

    // Set up write buffer for attribute
    const H5std_string strwritebuf ("This attribute is of type StrType");

    // Create attribute and write to it
    Attribute myatt_in = dataset.createAttribute(ATTR_NAME, strdatatype, attr_dataspace);
    myatt_in.write(strdatatype, strwritebuf);

    // Set up read buffer for attribute
    H5std_string strreadbuf ("");

    // Open attribute and read its contents
    Attribute myatt_out = dataset.openAttribute(ATTR_NAME);
    myatt_out.read(strdatatype, strreadbuf);

    // Display attribute contents
    cout << "Attribute contents: " << strreadbuf << endl;

    return 0;
}
