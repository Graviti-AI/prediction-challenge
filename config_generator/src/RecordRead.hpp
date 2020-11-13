//
// Created by lcr on 9/23/20.
//

#ifndef RECORDREAD_H
#define RECORDREAD_H

#include "car.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

class RecordRead {

public:
    RecordRead(std::string MapName,std::string TrackNumber);

    void Read(std::string CSV_file);

    std::vector<car*> data_car;
    std::string MapName_;
    std::string TrackNumber_ = "000";

    template <class Type>
    Type stringToNum(const std::string& str)
    {
        istringstream iss(str);
        Type num;
        iss >> num;
        return num;
    }

};


#endif //RECORDREAD_H