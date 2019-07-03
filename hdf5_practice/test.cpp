#include <stdio.h>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>

typedef struct SubjectMetadata{
    std::string Name;
    int Number;
    int Age;
    int Height;
    double Weight;
    std::string Gender;
    std::string H5File;
};


int ParseSubjectMetadataFile(SubjectMetadata & smds, std::string filepath){
    /* Determine if file exists */
    if (FILE * file = fopen(filepath.c_str(), "r")){
        fclose(file);
    }
    else{
        printf("File Not Found\nFile: %s \n", filepath.c_str());
        printf("\nRESULTS WILL NOT BE RECORDED\n");
        return 1;
    }

    /* Read in and parse file */
    std::ifstream file(filepath.c_str());
    // int maxlen = 256;
    // char buff[maxlen];
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
        // file.getline(buff, maxlen);
        // std::string line(buff);
        size_t ind = line.find(":");

        if (ind == std::string::npos){ /* Line does not have ":" delimiter */
            printf("Line does not have correct form\n");
            printf("Inputted line: %s\n", line.c_str());
            printf("Correct form:\nkey: value\n");
            printf("Exiting routine\n");
            // exit(1);
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
            }
            else if (key.compare("NUMBER")){
				smds.Number = atoi(valstr.c_str());
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
            }
            else{
                printf("Key does not match anything --- SKIPPING\n");
                printf("Line: %s\n", line.c_str());
                printf("Key: %s\n\n", key.c_str());
            }
        }
    }
    return 0;
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
	ParseSubjectMetadataFile(smds, sub_meta_file_str);
}
