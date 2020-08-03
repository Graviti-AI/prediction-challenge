//
// Created by 雷宇恒 on 2018/8/11.
//

#ifndef QUADPROG_QPSETUP_H
#define QUADPROG_QPSETUP_H

#include "Array.hh"


void QPSetup(double*, double*, double, unsigned int, unsigned int, quadprogpp::Matrix<double>&, quadprogpp::Matrix<double>&,
             quadprogpp::Matrix<double>&, quadprogpp::Vector<double>&, quadprogpp::Vector<double>&, quadprogpp::Vector<double>&);

#endif //QUADPROG_QPSETUP_H
