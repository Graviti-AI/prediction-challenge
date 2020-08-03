//
// Created by msclab on 10/31/18.
//

#ifndef AGENTSIM_REALCARMODEL_H
#define AGENTSIM_REALCARMODEL_H
#include "Model.hpp"

///
/// Use forward declaration to avoid including the header files of the complex model.
class Car_4wheel;

///
/// model for the real car
class RealCarModel : public Model {
public:
    explicit RealCarModel();
    virtual ~RealCarModel();

    Vector update(Vector state, Vector intermediate) override;


private:
    Car_4wheel *innerModel;
    long int count;
};
#endif //AGENTSIM_REALCARMODEL_H
