#include <iostream>
#include<Eigen/Dense>
#include<vector>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
using namespace std;
using namespace Eigen;

class MDN
{
    public:
    MDN();
    void get_likelihood(Eigen::MatrixXd& Input_Speed, Eigen::MatrixXd& Input_History, double& likelihood);

    private:
    MatrixXd WG;
    MatrixXd bG;
    MatrixXd Wc;
    MatrixXd bc;
    MatrixXd W0;
    MatrixXd W1;
    MatrixXd W2;
    MatrixXd b0;
    MatrixXd b1;
    MatrixXd b2;
    string path_W0;
    string path_b0;
    string path_W1;
    string path_b1;
    string path_W2;
    string path_b2;
    string path_Wc;
    string path_bc;
    string path_WG;
    string path_bG;

    void sigmoid(const Eigen::MatrixXd& Z, Eigen::MatrixXd& A, std::vector<Eigen::MatrixXd>& cache);

    void relu(const Eigen::MatrixXd& Z, Eigen::MatrixXd& A, std::vector<Eigen::MatrixXd>& cache);

    void tanh(const Eigen::MatrixXd& Z, Eigen::MatrixXd& A, std::vector<Eigen::MatrixXd>& cache);

    void Soft_max(const Eigen::MatrixXd& Z, Eigen::MatrixXd& A, std::vector<Eigen::MatrixXd>& cache);

    void linear_forward(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W, const Eigen::VectorXd& b, Eigen::MatrixXd& Z, std::vector<Eigen::MatrixXd>& cache);

    void linear_activation_forward(const Eigen::MatrixXd& A_prev, const Eigen::MatrixXd& W, const Eigen::VectorXd& b, const std::string activation, Eigen::MatrixXd& A, std::vector<std::vector<Eigen::MatrixXd> >& cache);

    void GRU_Cell(
        const Eigen::MatrixXd& c_prev, Eigen::MatrixXd& x, Eigen::MatrixXd& c, Eigen::MatrixXd& a, 
        const Eigen::MatrixXd& WG, const Eigen::VectorXd& bG,  const Eigen::MatrixXd& Wc,const Eigen::MatrixXd& bc
    );

    void GRU_Foward(
        Eigen::MatrixXd& x, Eigen::MatrixXd& c, Eigen::MatrixXd& a, 
        const Eigen::MatrixXd& WG, const Eigen::MatrixXd& bG, const Eigen::MatrixXd& Wc,const Eigen::MatrixXd& bc
    );

    void FC_3(
        Eigen::MatrixXd& Input, Eigen::MatrixXd& Output, const Eigen::MatrixXd& W0, const Eigen::MatrixXd& W1, const Eigen::MatrixXd& W2,
	    const Eigen::VectorXd& b0,  const Eigen::VectorXd& b1,  const Eigen::VectorXd& b2
    );

    void Post_process(Eigen::MatrixXd& Input, Eigen::MatrixXd& Output);

    void MDN_Model(
        Eigen::MatrixXd& Input, Eigen::MatrixXd& Output,
        const Eigen::MatrixXd& WG, const Eigen::VectorXd& bG, const Eigen::MatrixXd& Wc,const Eigen::MatrixXd& bc,
        const Eigen::MatrixXd& W0, const Eigen::MatrixXd& W1, const Eigen::MatrixXd& W2, const Eigen::VectorXd& b0, const Eigen::VectorXd& b1, const Eigen::VectorXd& b2
    );

    void load_txt(const std::string& path, Eigen::MatrixXd& Parameter);

    void load_Parameter(
        const std::string& path_WG, Eigen::MatrixXd& WG, const std::string& path_bG, Eigen::MatrixXd& bG,
        const std::string& path_Wc, Eigen::MatrixXd& Wc, const std::string& path_bc, Eigen::MatrixXd& bc,
        const std::string& path_W0, Eigen::MatrixXd& W0, const std::string& path_b0, Eigen::MatrixXd& b0,
        const std::string& path_W1, Eigen::MatrixXd& W1, const std::string& path_b1, Eigen::MatrixXd& b1,
        const std::string& path_W2, Eigen::MatrixXd& W2, const std::string& path_b2, Eigen::MatrixXd& b2
    );

    void pdf_mix2D_Gaussian(Eigen::MatrixXd& Input_Speed, double& likelihood, Eigen::MatrixXd& Parameter);


};