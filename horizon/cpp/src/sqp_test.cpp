#include "sqp.h"
#include <casadi/casadi.hpp>

int main()
{
    auto x = casadi::SX::sym("x", 1);
    auto u = casadi::SX::sym("u", 1);

    auto f = 3.*x*x + 2.*u;
    auto g = casadi::SX::vertcat({x, u});

    horizon::SQPGaussNewton<casadi::SX> sqp("sqp", "stocazzo", f, g, casadi::SX::vertcat({x, u}));

    Eigen::VectorXd x0(2);
    x0 << 2., 3.;
    sqp.solve(x0, x0, x0, x0, x0);
}
