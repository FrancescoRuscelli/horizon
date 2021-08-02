#include "ilqr.h"

int main()
{
    auto x = casadi::SX::sym("x", 3);
    auto u = casadi::SX::sym("u", 2);
    auto A = casadi::DM::rand(3, 3);
    auto B = casadi::DM::rand(3, 2);
    casadi::Function f("f", {x, u}, {casadi::SX::mtimes(A, x)});
    casadi_utils::WrappedFunction wf(f.factory("df", {"i0", "i1"}, {"jac:o0:i0", "jac:o0:i1"}));

    Eigen::VectorXd xval(3), uval(2);
    xval << 1, 0, 0;
    uval << 0, 1;

    wf.setInput(0, xval);
    wf.setInput(1, uval);
    wf.call();

    std::cout << A << std::endl;
    std::cout << B << std::endl;

    std::cout << wf.getOutput(0) << std::endl;
    std::cout << wf.getOutput(1) << std::endl;
}
