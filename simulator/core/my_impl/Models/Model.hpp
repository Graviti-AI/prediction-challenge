//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_MODEL_HPP
#define AGENTSIM_MODEL_HPP

#include <vector>
#include <ctime>
typedef std::vector<double> Vector;

///
/// Parent class for all models.
class Model {
public:
    explicit Model(int dimState, int dimIntermediate);

    virtual Vector update(Vector state, Vector intermediate) = 0;

protected:
    std::clock_t last_time;
    const int dimState;  /*!< dimension of the state vector.*/
    const int dimIntermediate;  /*!< dimension of the intermediate vector.*/
};


#endif //AGENTSIM_MODEL_HPP
