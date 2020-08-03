//
// Created by msclab on 10/31/18.
//

#include "RealCarModel.hpp"

#include <iostream>
#include <fstream>

#include "FourWheelModel/Car_4wheel.h"
using namespace std;
/// Constructor.
/// Dimension of state vector is 6 (x, y, yaw, speed x, speed y, speed yaw)
/// Dimension of intermediate vector is 3 (proportional to gas pedal, brake pedal, and steering angle)
RealCarModel::RealCarModel() : Model(6, 3) {
    this->innerModel = new Car_4wheel();
    this->count = 0;
}

/// Wrapper for the complex model
/// \param state Dimension of state vector is 6 (x, y, yaw, speed x, speed y, speed yaw)
/// \param intermediate Dimension of intermediate vector is 3 (proportional to gas pedal, brake pedal, and steering angle)
/// \return the state vector of this car for the next iteration
Vector RealCarModel::update(Vector state, Vector intermediate) {

    string name[18];
    int index[19];
    double number[18];
    std::string writebuf;
    int length;
    ifstream in("/home/mscsim/mkz-mpc-control/a.txt");
    if(in) // the file exists
    {
        while (getline(in, writebuf))
        {
            length = writebuf.length();
            int c = 1;
            index[0] = -1;
            for (int b = 0; b < length; b++) {
                if (writebuf[b] == '&' || writebuf[b] == '|') {
                    index[c] = b;
                    name[c - 1] = writebuf.substr(index[c - 1] + 1, index[c]);
                    number[c - 1] = atof(const_cast<const char *>(name[c - 1].c_str()));
                    cout << "precision: " << number[c - 1] << number[c - 1] * 100 << endl;
                    c++;
                }

            }
        }
    }
    else
    {
        for(int ii = 0; ii < 6; ii++)
        {
            number[ii] = 0.0;
        }
    }

    // std::cout << this->innerModel->output[0] << " " << this->innerModel->output[1] << " " << this->innerModel->output[2] << std::endl;


    return Vector{
            number[2] * 100,
            number[3] * 100,
            number[4],
            number[0] * 100,
            number[1] * 100,
            number[5]
    };

}

RealCarModel::~RealCarModel() {
    delete this->innerModel;
}
