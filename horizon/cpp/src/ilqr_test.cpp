#include "ilqr.h"

int main()
{
    auto x = casadi::SX::sym("x", 1);
    auto u = casadi::SX::sym("u", 1);
    auto f = casadi::Function("f", {x, u}, {x + 0.1*u}, {"x", "u"}, {"f"});
    auto l = casadi::Function("l", {x, u}, {casadi::SX::sumsqr(u)*1e-6}, {"x", "u"}, {"l"});
    auto lf = casadi::Function("l", {x, u}, {casadi::SX::sumsqr(x)*0.5}, {"x", "u"}, {"l"});
    auto cf = casadi::Function("h", {x, u}, {x - 1}, {"x", "u"}, {"h"});

    int N = 2;
    horizon::IterativeLQR ilqr(f, N);

    Eigen::VectorXd x0(1);
    x0 << 0.0;
    ilqr.setInitialState(x0);

    ilqr.setIntermediateCost(std::vector<casadi::Function>(N, l));
//    ilqr.setFinalCost(lf);
    ilqr.setFinalConstraint(cf);

    ilqr.solve(1);

    std::cout << ilqr.getStateTrajectory() << std::endl;
    std::cout << ilqr.getInputTrajectory() << std::endl;
}
