/**
 * This file is used to set the needed feature and their weights 
 */
#if !defined(FEATURELIST_H)
#define FEATURELIST_H

/**
 * Define the features and weights
 *  To delete or add features or change the weight, the following table is the only place needed to change.
 */
#define FEATURE_WEIGHT_TABLE(PRE,F) \
    F(PRE, L2_a_lon, 1.51870706e-01) \
    F(PRE, L2_j_lon, 2.15838655e-01) \
    F(PRE, L2_a_lat, 6.33729680e-02) \
    F(PRE, L2_abs_v_des, 5.80264235e-05) \
    F(PRE, future_dist, 2.87465306e-03) \
    F(PRE, future_inter_dist, 7.53269570e-05) \

/** The result of experiment `DoubleCarTrain_Already-in-car-4-13_2`
 * L2_a_lon:0.06709603445259091
 * L2_a_lat:0.5954397385822132
 * L2_j_lon:0.33693295422881364
 * L2_abs_v_des:0.00026257388302037296
 * L1_future_distance:0.00025751308799036273
 * L1_future_inter_dist:1.1185765371291012e-05
 * 
 * Normalization of each feature (The std of features)
 *  0.44179708 2.75872614 5.31666679 4.52507439 0.08958058 0.14849618  
 * 
 * So the combining the weight and std, the weight is:
 * L2_a_lon: 1.51870706e-01
 * L2_a_lat: 2.15838655e-01
 * L2_j_lon: 6.33729680e-02
 * L2_abs_v_des: 5.80264235e-05
 * L1_future_distance: 2.87465306e-03
 * L1_future_inter_dist: 7.53269570e-05
 */

/**
 * Define the feature calculation flags and values
 */
#define dt 0.1
#define vlim 4.76
#define Futuredt 1
#define FUT_DIST_COEFF -0.04
#define FUT_INTER_DIST_COEFF -0.8
//***** Comment out the following flags means set them to False *****//
// #define L2NotSquared
#define WHOLEPROCESSNORM

// util macros to simplify code
#define DECLARE_FTR_VEC(PRE, NAME, WEIGHT)  double NAME;
#define ASSIGN_FTR_VEC(PRE, NAME, WEIGHT) PRE NAME = NAME;
#define PRINT_FTRS(PRE, NAME,WEIGHT) std::cout<<#NAME << ": " <<PRE NAME<<std::endl;

#define SUM_VALUE(PRE, NAME, WEIGHT)   + PRE NAME * WEIGHT



// #define CONSTRUCT_FTR_TYPE(F)\

#endif // FEATURELIST_H
