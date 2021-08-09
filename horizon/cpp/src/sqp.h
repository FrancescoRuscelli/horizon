#include <casadi/casadi.hpp>
#include "wrapped_function.h"
#include <Eigen/Dense>
#include <memory>


#include "profiling.h"

namespace horizon{

typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;
typedef Eigen::Ref<const Eigen::MatrixXd> MatConstRef;

template <class CASADI_TYPE>
class SQPGaussNewton
{
public:

    SQPGaussNewton(const std::string& name, const std::string& qp_solver,
                   const CASADI_TYPE& f, const CASADI_TYPE& g, const CASADI_TYPE& x):
        _name(name), _qp_solver(qp_solver),
        _x(x),
        _max_iter(1000),
        _reinitialize_qp_solver(false)
    {
        _f = casadi::Function("f", {_x}, {f}, {"x"}, {"f"});
        _df = _f.function().factory("df", {"x"}, {"jac:f:x"});


        _g = casadi::Function("g",{_x}, {g}, {"x"}, {"g"});
        _dg = _g.function().factory("dg", {"x"}, {"jac:g:x"});

    }

    bool solve(const Eigen::VectorXd& x0, const Eigen::VectorXd& lbx, const Eigen::VectorXd& ubx,
               const Eigen::VectorXd& lbg, const Eigen::VectorXd& ubg, const double alpha=1.)
    {
        Eigen::VectorXd v_opt = x0;

        bool sparse = true;

        _f.setInput(0, x0);
        _f.call();

        _df.setInput(0, x0);
        _df.call(sparse);

        _g.setInput(0, x0);
        _g.call();

        _dg.setInput(0, x0);
        _dg.call(sparse);

        Eigen::VectorXd f = _f.getOutput(0);
        Eigen::VectorXd g = _g.getOutput(0);

        Eigen::SparseMatrix<double> Jf = _df.getSparseOutput(0);
        Eigen::SparseMatrix<double> Jg = _dg.getSparseOutput(0);

        // Gauss-Newton Hessian
        Eigen::SparseMatrix<double> H(Jf.cols(), Jf.cols());
        //H.triangularView<Eigen::Upper>() = Jf.transpose()*Jf; <-- DO NOT COMPILE!
        //H = H.selfadjointView<Eigen::Upper>();
        H = Jf.transpose()*Jf; //<-- to optimize

        Eigen::VectorXd grad = Jf.transpose()*f;



        //casadi::Function qp = casadi::conic("qp_solver", "osqp", );


    }


private:


    CASADI_TYPE _x;
    std::string _name;
    std::string _qp_solver;

    // dynamics function
    casadi_utils::WrappedFunction _f, _g;

    // dynamics jacobian
    casadi_utils::WrappedFunction _df, _dg;


    int _max_iter;
    bool _reinitialize_qp_solver;





};

}
