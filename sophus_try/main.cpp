#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "sophus/so3.hpp"

template class Eigen::Quaternion<float>;
template class Eigen::Quaternion<double>;

int main()
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXd m = Eigen::MatrixXd::Random(3,3);
    m = (m + Eigen::MatrixXd::Constant(3,3,1.2)) * 50;
    std::cout << "m =" << std::endl << m << std::endl;
    Eigen::VectorXd v(3);
    v << 1, 2, 3;
    std::cout << "m * v =" << std::endl << m * v << std::endl;
    Eigen::Quaternion<double> myQuat(0.5, 0.5, 0.5, 0.5);
    std::cout<<myQuat.x()<<std::endl;
    
    std::vector<Sophus::SO3Group<float>> so3_vec;
    
    so3_vec.push_back(Sophus::SO3Group<float>(Eigen::Quaternion<float>(0.1e-11, 0., 1., 0.)));
    so3_vec.push_back(Sophus::SO3Group<float>(Eigen::Quaternion<float>(-1,0.00001,0.0,0.0)));
    so3_vec.push_back(Sophus::SO3Group<float>::exp(Sophus::SO3Group<float>::Point(0.2, 0.5, 0.0)));
    
}