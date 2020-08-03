#ifndef FEATURE_H_
#define FEATURE_H_

/**
 * The feature calculation code. 
 * For the time being, the structure of this code is quite inefficient, 
 *      there are quite a lot of computations that can be reused.
 */

#include <Eigen/Dense>
#include <Eigen/Core>
#include <utility> // pair
#include "featureList.h"


namespace likelical
{

using vector = Eigen::VectorXd;
using traj = Eigen::MatrixX2d;
using point = Eigen::RowVector2d;
struct featureVec
{
    /**
     * The structure to hold a value for each feature
     */
    FEATURE_WEIGHT_TABLE(,DECLARE_FTR_VEC);
};

// interface function
/**
 * calcFeatures
 * param: egocar: Eigen::Matrix<double,-1,2>    The trajectory of the self car
 * param: othercar: Eigen::Matrix<double,-1,2>  The trajectory of the other car, should be equivalent length to egocar
 * param: egoref: Eigen::Matrix<double,-1,2>    The reference trajectory of the self car, no requirement on its length
 * param: otherref: Eigen::Matrix<double,-1,2>  The reference trajectory of the other car, no requirement on its length
 * param: interInd: pair<int,int>               (ego_inter_index, other_inter_index). 
 *                                               Where the inter_index means the index of the interaction point on reference trajectory
 */
featureVec calcFeatures(const traj& egocar, const traj& othercar, const traj& egoref, const traj& otherref, std:: pair<int,int> interInd);

// inner function
traj closestPoint(const traj& curv1, const traj& curv2, Eigen::VectorXi& refinds);
Eigen::VectorXi curveMatch(const traj& crv1, const traj& crv2);

} // namespace likelical

#endif //FEATURE_H_