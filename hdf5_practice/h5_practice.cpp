#ifdef OLD_HEADER_FILENAME
#include <iostream.h>
#else
#include <iostream>
#endif
#include <string>
#include <algorithm>
#include <fstream>
#include <memory>
#include "H5Cpp.h"
using namespace H5;


typedef struct SubjectMetadata{
    std::string Name;
    int Number;
    int Age = 0;
    int Height = 0;
    double Weight = 0;
    std::string Gender = "";
    std::string H5File;
};


bool ParseSubjectMetadataFile(SubjectMetadata & smds, std::string filepath){
    /* Determine if file exists */
    if (FILE * file = fopen(filepath.c_str(), "r")){
        fclose(file);
    }
    else{
        printf("File Not Found\nFile: %s \n", filepath.c_str());
        printf("\nRESULTS WILL NOT BE RECORDED\n");
        return false;
    }

    /* Read in and parse file */
    std::ifstream file(filepath.c_str());
    bool hasName = false;
    bool hasH5Filepath = false;
    bool hasSubjectNumber = false;
	std::string line;
    while (getline(file, line)){
        /*
        Each line is assumed to have the form key: value
        To process:
        1. Split at ":" delimiter
        2. Compare the key value
        3. Strip whitespace off value string
        4. Store value in SubjectMetadata struct
        */
        size_t ind = line.find(":");

        if (ind == std::string::npos){ /* Line does not have ":" delimiter */
            printf("Line does not have correct form\n");
            printf("Inputted line: %s\n", line.c_str());
            printf("Correct form:\nkey: value\n");
        }
        else{                          /* Line has ":" delimiter */
            /* Separate key and value from line */
            std::string key = line.substr(0, ind);
            std::string valstr = line.substr((ind+1), valstr.length() - (ind+1));
            /* Strip whitespace off front of valstr*/
            size_t first_char_pos = valstr.find_first_not_of(" ");
            valstr = valstr.substr(first_char_pos, valstr.length() - first_char_pos);

            /* Make key string uppercase so comparing is easier*/
            std::transform(key.begin(), key.end(), key.begin(), ::toupper);

            /* Compare key string and assign values to SubjectMetdata struct */
            if (key.compare("NAME")){
                smds.Name = valstr;
                hasName = true;
            }
            else if (key.compare("NUMBER")){
				smds.Number = atoi(valstr.c_str());
                hasSubjectNumber = true;
            }
            else if (key.compare("AGE")){
				smds.Age = atoi(valstr.c_str());
            }
            else if (key.compare("HEIGHT")){
				smds.Height = atoi(valstr.c_str());
			}
            else if (key.compare("WEIGHT")){
				smds.Weight = atof(valstr.c_str());
            }
            else if (key.compare("GENDER")){
				smds.Gender = valstr;
            }
            else if (key.compare("H5FILE")){
				smds.H5File = valstr;
                hasH5Filepath = true;
            }
            else{
                printf("Key does not match anything --- SKIPPING\n");
                printf("Line: %s\n", line.c_str());
                printf("Key: %s\n\n", key.c_str());
            }
        }
    }
    /* Return whether or not the file has enough info for H5 logging*/
    if (hasSubjectNumber && hasName && hasH5Filepath){
        return true;
    }
    else{
        printf("File found, but not enough information specified in file\n");
        printf("RESULTS WILL NOT BE RECORDED\n");
        return false;
    }
}


const H5std_string FILE_NAME("myhdf.h5");

typedef struct TrialDatasets{
    DataSet JointAngle;      /* Nx7 element dataspace of type double */
    DataSet Force;           /* Nx2 element dataspace of type double */
    DataSet PerturbFlag;     /* Nx1 element dataspace of type int    */
    DataSet EndEffectorPO;   /* Nx6 element dataspace of type double */
};

H5File * create_or_open_file(const std::string & filename){
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

Group * create_or_open_group(H5File * file, H5std_string & group_name){

    Exception::dontPrint();
    Group * group = 0;

    try{
        /* Open group if it exists */
        group = new Group(file->openGroup(group_name));
    } catch(Exception & e){
        group = new Group(file->createGroup(group_name));
    }

    return group;
}

TrialDatasets CreateTrialDatasets(H5Location * h5loc){
    TrialDatasets td;

    // Define dataspace dimensions
    hsize_t dims_ja[2] = {10000, 7};
    hsize_t dims_force[2] = {10000, 2};
    hsize_t dims_pf[2] = {10000, 1};
    hsize_t dims_eepo[2] = {10000, 6};
    hsize_t maxdims_ja[2] = {H5S_UNLIMITED, 7};
    hsize_t maxdims_force[2] = {H5S_UNLIMITED, 2};
    hsize_t maxdims_pf[2] = {H5S_UNLIMITED, 1};
    hsize_t maxdims_eepo[2] = {H5S_UNLIMITED, 6};

    // Define dataspaces
    DataSpace dataspace_ja = DataSpace(2, dims_ja, maxdims_ja);
    DataSpace dataspace_force = DataSpace(2, dims_force, maxdims_force);
    DataSpace dataspace_pf = DataSpace(2, dims_pf, maxdims_pf);
    DataSpace dataspace_eepo = DataSpace(2, dims_eepo, maxdims_eepo);

    // Property List
    DSetCreatPropList prop_ja;
    DSetCreatPropList prop_force;
    DSetCreatPropList prop_pf;
    DSetCreatPropList prop_eepo;

    prop_ja.setChunk(2, dims_ja);
    prop_force.setChunk(2, dims_force);
    prop_pf.setChunk(2, dims_pf);
    prop_eepo.setChunk(2, dims_eepo);

    // Create Datasets
    td.JointAngle = h5loc->createDataSet("JointAngle",
                                         PredType::NATIVE_DOUBLE,
                                         dataspace_ja,
                                         prop_ja);
    td.Force = h5loc->createDataSet("Force",
                                    PredType::NATIVE_DOUBLE,
                                    dataspace_force,
                                    prop_force);
    td.PerturbFlag = h5loc->createDataSet("PerturbFlag",
                                          PredType::NATIVE_INT,
                                          dataspace_pf,
                                          prop_pf);
    td.EndEffectorPO = h5loc->createDataSet("EndEffectorPO",
                                            PredType::NATIVE_DOUBLE,
                                            dataspace_eepo,
                                            prop_eepo);

    return td;
}

void CreateStringAttribute(H5Object * h5obj, H5std_string attr_name, H5std_string attr_val){
    // Create new dataspace for attribute -- H5S_SCALAR means the dataspace has 1 element
    DataSpace ds = DataSpace(H5S_SCALAR);

    // Create new string datatype for attribute
    StrType strdatatype(PredType::C_S1, 256); // of length 256 characters

    // Create attribute and write to it
    Attribute attr = h5obj->createAttribute(attr_name, strdatatype, ds);
    attr.write(strdatatype, attr_val);
}

void CreateNumericAttribute(H5Object * h5obj, H5std_string attr_name, double attr_val){
    // Create new dataspace for attribute -- H5S_SCALAR means the dataspace has 1 element
    DataSpace ds = DataSpace(H5S_SCALAR);

    // Create float datatype
    FloatType floatdatatype(PredType::NATIVE_DOUBLE);

    // Create attribute and write to it
    Attribute attr = h5obj->createAttribute(attr_name, floatdatatype, ds);
    attr.write(floatdatatype, &attr_val);
}

int main(int argc, char ** argv){

    /* Command Line Arguments */
    if (argc < 2){
        printf("Command Line Arguments: \n");
        printf("    1.  Subject Metadata File\n");
        exit(1);
    }

	SubjectMetadata smds;
	std::string sub_meta_file_str(argv[1]);
	bool h5active = ParseSubjectMetadataFile(smds, sub_meta_file_str);

    if (h5active){
        /* Initialize H5 objects */
        Exception::dontPrint();
        H5File * file = 0;
        Group * group_subject = 0;
        Group * group_trial = 0;

        /* File */
        file =  create_or_open_file(FILE_NAME);

        /* Group - Subject */
        H5std_string group_subject_name = std::string("Subject") + std::string(argv[1]);
        group_subject = create_or_open_group(file, group_subject_name);

        /* Group - Trial */
        H5std_string group_trial_name = group_subject_name + "/" + std::string("Trial") + std::string("1");
        group_trial = create_or_open_group(file, group_trial_name);

        /* /Subject group attributes */
        CreateStringAttribute(group_subject, "Name", "Tanner Bitz");
        CreateNumericAttribute(group_subject, "Age", 25);
        CreateStringAttribute(group_subject, "Gender [M/F]", "M");
        CreateNumericAttribute(group_subject, "Height [in]", 69);
        CreateNumericAttribute(group_subject, "Weight [lbs]", 168);

        /* /Subject/Trial group attributes */
        CreateNumericAttribute(group_trial, "x_target", 1);
        CreateNumericAttribute(group_trial, "y_target", 2);
        CreateNumericAttribute(group_trial, "Damping", -5);

        TrialDatasets td = CreateTrialDatasets(group_trial);
        
    }



    /* Close */
    file->close();
    printf("file closed\n");

}
