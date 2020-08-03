#ifndef LIKELICAL_H_
#define LIKELICAL_H_
#include "feature.h"
#include "featureList.h"
namespace likelical
{
    /**
     * likelihood
     * param: egocar: Eigen::Matrix<double,-1,2>    The trajectory of the self car
     * param: othercar: Eigen::Matrix<double,-1,2>  The trajectory of the other car, should be equivalent length to egocar
     * param: egoref: Eigen::Matrix<double,-1,2>    The reference trajectory of the self car, no requirement on its length
     * param: otherref: Eigen::Matrix<double,-1,2>  The reference trajectory of the other car, no requirement on its length
     * param: interInd: pair<int,int>               (ego_inter_index, other_inter_index). 
     *                                               Where the inter_index means the index of the interaction point on reference trajectory
     */
    // double likelihood(const traj& egocar, const traj& othercar, const traj& egoref, const traj& otherref, std:: pair<int,int> interInd);
    double likelihood(const traj& egocar, const traj& othercar, 
            const traj& egoref, const traj& otherref, std:: pair<int,int> interInd, std:: pair<int,int> interInd_obstacle);


} // namespace likelical



#endif //LIKELICAL_H_