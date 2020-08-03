//
// Created by mscsim on 8/23/18.
//

#ifndef AGENTSIM_CONTROLLER_HPP
#define AGENTSIM_CONTROLLER_HPP

#include<vector>
typedef std::vector<double> Vector;

///
/// Parent class of all controllers.
class Controller {
public:
    explicit Controller(int dimInput, int dimIntermediate);

    virtual Vector update(Vector input) = 0;

protected:
    const int dimInput; /*!< dimension of the input vector.*/
    const int dimIntermediate; /*!< dimension of the intermediate vector.*/
};


#endif //AGENTSIM_CONTROLLER_HPP
