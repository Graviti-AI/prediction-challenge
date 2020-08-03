#include<Eigen/Dense>
#include<cmath>
#include <iostream>
#include <fstream>
using namespace Eigen;
using namespace std;

class GMM{
public:
  GMM();
  ~GMM(){cout<<"end of prediction";};
  double get_pdf(double current_state_data[],double future_state_data[]);
private:
  Matrix<double, 1, 8> GLOBAL_MEANS;
  Matrix<double, 1,6> mu_i_for_getting_weights;
  Matrix<double, 1,2> mu_o_for_getting_means;
  Matrix<double, 8,8> GLOBAL_COV;
  Matrix<double, 6,6> sigma_i_for_getting_weights;
  Matrix<double, 2,6> sigma_oi;
  Matrix<double, 6,2> sigma_io;
  Matrix<double, 2,2> sigma_o;
  Matrix<double, 6,6> sigma_i_inverse;
};

// struct parameter{
//   Matrix<double, 1, 8> GLOBAL_MEANS;
//   Matrix<double, 1,6> mu_i_for_getting_weights;
//   Matrix<double, 1,2> mu_o_for_getting_means;
//   Matrix<double, 8,8> GLOBAL_COV;
//   Matrix<double, 6,6> sigma_i_for_getting_weights;
//   Matrix<double, 2,6> sigma_oi;
//   Matrix<double, 6,2> sigma_io;
//   Matrix<double, 2,2> sigma_o;
//   Matrix<double, 6,6> sigma_i_inverse;
// };



