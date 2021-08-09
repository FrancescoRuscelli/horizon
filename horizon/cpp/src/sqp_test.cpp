#include "sqp.h"
#include <casadi/casadi.hpp>

int main()
{

    Eigen::VectorXd v(5);
    v << 1., 2., 3., 4., 5.;

    casadi::DM v_DM;
    casadi_utils::toCasadiMatrix(v, v_DM);

    std::cout<<"v: "<<v.transpose()<<std::endl;
    std::cout<<"v_DM: "<<v_DM<<std::endl;

    Eigen::MatrixXd I; I.setIdentity(3,3);

    casadi::DM I_DM;
    casadi_utils::toCasadiMatrix(I, I_DM);

    std::cout<<"I: \n"<<I<<std::endl;
    std::cout<<"I_DM: "<<I_DM<<std::endl;

    Eigen::MatrixXd M(2,3);
    M<<1.,2.,3.,
       4.,5.,6.;

    casadi::DM M_DM;
    casadi_utils::toCasadiMatrix(M, M_DM);

    std::cout<<"M: \n"<<M<<std::endl;
    std::cout<<"M_DM: "<<M_DM<<std::endl;



//    auto x = casadi::SX::sym("x", 1);
//    auto u = casadi::SX::sym("u", 1);

//    auto f = 3.*x*x + 2.*u;
//    auto g = casadi::SX::vertcat({x, u});

//    horizon::SQPGaussNewton<casadi::SX> sqp("sqp", "stocazzo", f, g, casadi::SX::vertcat({x, u}));

//    Eigen::VectorXd x0(2);
//    x0 << 2., 3.;
//    sqp.solve(x0, x0, x0, x0, x0);
}
