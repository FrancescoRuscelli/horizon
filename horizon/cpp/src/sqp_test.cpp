#include "sqp.h"
#include <casadi/casadi.hpp>

int main()
{
    auto x = casadi::SX::sym("x", 1);
    auto u = casadi::SX::sym("u", 1);

    auto f = 3.*x*x + 2.*u;
    auto g = casadi::SX::vertcat({x, u});

    casadi::Dict opts;
    opts["max_iter"] = 5;
    opts["printLevel"] = "none";
    horizon::SQPGaussNewton<casadi::SX> sqp("sqp", "qpoases", f, g, casadi::SX::vertcat({x, u}), opts);

    casadi::DM x0(2,1);
    x0(0) = 2.;
    x0(1) = 3.;

    casadi::DM lb(2,1), ub(2,1);
    lb(0) = 0.;
    lb(1) = -1.;
    ub(0) = 0.;
    ub(1) = 2.;
    auto solution = sqp.solve(x0, lb, ub, lb, ub);

    std::cout<<"solution: "<<solution["x"]<<std::endl;

    casadi::DMVector var_trj = sqp.getVariableTrajectory();
    for(unsigned int i = 0; i < var_trj.size(); ++i)
        std::cout<<"iter "<<i<<": "<<var_trj[i]<<std::endl;
}
