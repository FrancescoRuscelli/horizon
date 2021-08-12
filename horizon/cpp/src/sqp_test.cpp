#include "sqp.h"
#include <casadi/casadi.hpp>

int main()
{
    auto x = casadi::SX::sym("x", 2);
    auto u = casadi::SX::sym("u", 1);

    auto f = 3.*x(0)*x(1)+ 2.*u;
    auto g = casadi::SX::vertcat({x, u});

    casadi::Dict opts;
    opts["max_iter"] = 5;
    opts["printLevel"] = "none";
    horizon::SQPGaussNewton<casadi::SX> sqp("sqp", "qpoases", f, g, casadi::SX::vertcat({x, u}), opts);

    casadi::DM x0(3,1);
    x0(0) = 2.;
    x0(1) = 3.;
    x0(2) = 4.;

    casadi::DM lb(3,1), ub(3,1);
    lb(0) = 1.5;
    lb(1) = -1.;
    lb(1) = -2.;
    ub(0) = 1.5;
    ub(1) = 2.;
    ub(1) = 2.;
    auto solution = sqp.solve(x0, lb, ub, lb, ub);

    std::cout<<"solution: "<<solution["x"]<<"   f: "<<solution["f"]<<"  g: "<<solution["g"]<<std::endl;

    casadi::DMVector var_trj = sqp.getVariableTrajectory();
    std::vector<double> objs = sqp.getObjectiveIterations();
    std::vector<double> cons = sqp.getConstraintNormIterations();
    for(unsigned int i = 0; i < var_trj.size(); ++i)
        std::cout<<"iter "<<i<<"-> sol: "<<var_trj[i]<<"    obj: "<<objs[i]<<"  cons: "<<cons[i]<<std::endl;
}
