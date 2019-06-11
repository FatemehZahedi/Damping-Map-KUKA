#include <fstream>
#include <algorithm>
#include <string>
#include <stdio.h>


namespace SubjectMetadataParser{


    typedef struct SubjectMetadata{
        std::string Name;
        int Number;
        int Age = 0;
        int Height = 0;
        double Weight = 0;
        std::string Gender = "";
        std::string H5File;
    };


    bool ParseSubjectMetadataFile(SubjectMetadata & smds, std::string filepath);

}
