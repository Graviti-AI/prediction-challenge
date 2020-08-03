//
// Created by mscsim on 8/29/18.
//

#ifndef AGENTSIM_FOURWHEELMODEL_HPP
#define AGENTSIM_FOURWHEELMODEL_HPP

#include "Model.hpp"

///
/// Use forward declaration to avoid including the header files of the complex model.
class Car_4wheel;

///
/// Complex model for a human car
class FourWheelModel : public Model {
public:
    explicit FourWheelModel();
    virtual ~FourWheelModel();

    Vector update(Vector state, Vector intermediate) override;


private:
    Car_4wheel *innerModel;
    long int count;
};


#endif //AGENTSIM_FOURWHEELMODEL_HPP
