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
    struct IODMDict{
        casadi::DMDict input;
        casadi::DMDict output;
    };


    SQPGaussNewton(const std::string& name, const std::string& qp_solver,
                   const CASADI_TYPE& f, const CASADI_TYPE& g, const CASADI_TYPE& x, const casadi::Dict& opts = casadi::Dict()):
        _name(name), _qp_solver(qp_solver),
        _x(x),
        _max_iter(1000),
        _reinitialize_qp_solver(false),
        _opts(opts), _qp_opts(opts)
    {
        _f = casadi::Function("f", {_x}, {f}, {"x"}, {"f"});
        _df = _f.function().factory("df", {"x"}, {"jac:f:x"});


        _g = casadi::Function("g",{_x}, {g}, {"x"}, {"g"});
        _dg = _g.factory("dg", {"x"}, {"jac:g:x"});

        if(opts.contains("max_iter"))
        {
            _max_iter = opts.at("max_iter");
            _qp_opts.erase("max_iter");
        }

        if(opts.contains("reinitialize_qpsolver"))
        {
            _reinitialize_qp_solver = opts.at("reinitialize_qpsolver");
            _qp_opts.erase("reinitialize_qpsolver");
        }


        _variable_trj.resize(_max_iter+1, casadi::DM(x.rows(), x.columns()));

    }

    void print_options_conic(std::ostream &stream=casadi::uout()) const
    {
        if(_conic)
            _conic->print_options(stream);
    }

    const casadi::DMDict& solve(const casadi::DM& initial_guess_x, const casadi::DM& lbx, const casadi::DM& ubx,
                                const casadi::DM& lbg, const casadi::DM& ubg, const double alpha=1.)
    {
        bool sparse = true;
        casadi::DM x0 = initial_guess_x;
        Eigen::VectorXd sol;
        casadi_utils::toEigen(x0, sol);
        _variable_trj[0] = x0;
        for(unsigned int k = 0; k < _max_iter; ++k)
        {
            //1. Cost function is linearized around actual x0
            _f.setInput(0, sol); // cost function
            _f.call();
            Eigen::VectorXd f = _f.getOutput(0);

            _df.setInput(0, sol); // cost function Jacobian
            _df.call(sparse);
            Eigen::SparseMatrix<double> J = _df.getSparseOutput(0);

            //2. Constraints are linearized around actual x0
            IODMDict g_dict;
            g_dict.input["x"] = x0;
            _g.call(g_dict.input, g_dict.output);

            IODMDict A_dict;
            A_dict.input["x"] = x0;
            _dg.call(A_dict.input, A_dict.output);

            casadi::DM g = g_dict.output["g"];
            casadi::DM A = A_dict.output["jac_g_x"];

            //2. We compute Gauss-Newton Hessian approximation and gradient function
            Eigen::SparseMatrix<double> H(J.cols(), J.cols());
            H = J.transpose()*J; //<-- to optimize

            Eigen::VectorXd grad = J.transpose()*f;

            //3. Setup QP
            casadi::DM grad_;
            casadi_utils::toCasadiMatrix(grad, grad_);

            ///TODO: Optimize using directly sparsity
            casadi::DM H_;
            casadi_utils::toCasadiMatrix(H.toDense(), H_);

            if(!_conic || _reinitialize_qp_solver)
            {
                std::cout<<"init QP"<<std::endl;
                _conic_init_input["h"] = H_.sparsity();
                _conic_init_input["a"] = A.sparsity();
                _conic = std::make_unique<casadi::Function>(casadi::conic("qp_solver", _qp_solver, _conic_init_input, _qp_opts));
            }

            _conic_dict.input["h"] = H_;
            _conic_dict.input["g"] = grad_;
            _conic_dict.input["a"] = A;
            _conic_dict.input["lba"] = lbg - g;
            _conic_dict.input["uba"] = ubg - g;
            _conic_dict.input["lbx"] = lbx - x0;
            _conic_dict.input["ubx"] = ubx - x0;
            _conic_dict.input["x0"] = x0;

            _conic->call(_conic_dict.input, _conic_dict.output);

            //4. Take full step
            x0 = x0 + alpha*_conic_dict.output["x"];
            casadi_utils::toEigen(x0, sol);


            // store trajectory
            _variable_trj[k+1] = x0;
        }

        _solution["x"] = x0;
        return _solution;
    }

    const casadi::DMVector& getVariableTrajectory() const
    {
        return _variable_trj;
    }

private:


    CASADI_TYPE _x;
    std::string _name;
    std::string _qp_solver;

    // Cost function and Jacobian
    casadi_utils::WrappedFunction _f, _df;

    // Constraint and Jacobian
    casadi::Function _g, _dg;


    int _max_iter;
    bool _reinitialize_qp_solver;

    std::unique_ptr<casadi::Function> _conic;
    casadi::SpDict _conic_init_input;
    IODMDict _conic_dict;

    casadi::DMDict _solution;

    casadi::Dict _opts;
    casadi::Dict _qp_opts;

    casadi::DMVector _variable_trj;


};

}
