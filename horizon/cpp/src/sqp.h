#ifndef __HORIZON__SQP__H__
#define __HORIZON__SQP__H__

#include <casadi/casadi.hpp>
#include "wrapped_function.h"
#include <Eigen/Dense>
#include <memory>


#include "profiling.h"

namespace horizon{

typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;
typedef Eigen::Ref<const Eigen::MatrixXd> MatConstRef;

template <class CASADI_TYPE> ///casadi::SX or casadi::MX
class SQPGaussNewton
{
public:
    struct IODMDict{
        casadi::DMDict input;
        casadi::DMDict output;
    };

    /**
     * @brief SQPGaussNewton SQP method with Gauss-Newton approximaiton (Hessian = J'J)
     * @param name solver name
     * @param qp_solver name internally used with casadi::conic (check casadi documetation for conic)
     * @param f cost function as casadi::Function with single input and single output (otherwise throw)
     * @param g constraints as casadi::Function with single input and single output (otherwise throw)
     * @param opts options for SQPGaussNewton and internal conic (check casadi documetation for conic).
     * NOTE: options for SQPGaussNewton are:
     *                                          "max_iter": iterations used to find solution
     *                                          "reinitialize_qpsolver": if true the internal qp solver is initialized EVERY iteration
     */
    SQPGaussNewton(const std::string& name, const std::string& qp_solver,
                   const casadi::Function& f, const casadi::Function& g, const casadi::Dict& opts = casadi::Dict()):
        _name(name), _qp_solver(qp_solver),
        _max_iter(1000),
        _reinitialize_qp_solver(false),
        _opts(opts), _qp_opts(opts),
        _alpha(1.)
    {

        if(f.n_in() != 1)
            throw std::runtime_error("Expected inputs for f is 1");
        if(f.n_out() != 1)
            throw std::runtime_error("Expected outputs for f is 1");
        if(g.n_in() != 1)
            throw std::runtime_error("Expected inputs for g is 1");
        if(g.n_out() != 1)
            throw std::runtime_error("Expected ouptuts for g is 1");


        _f = f;
        _df = f.factory("df", {f.name_in(0)}, {"jac:" + f.name_out(0) +":" + f.name_in(0)});

        _g = g;
        _dg = _g.factory("dg", {g.name_in(0)}, {"jac:" + g.name_out(0) + ":" + g.name_in(0)});

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


        _variable_trj.resize(_max_iter+1, casadi::DM(f.size1_in(0), f.size2_in(0)));
    }

    /**
     * @brief SQPGaussNewton SQP method with Gauss-Newton approximaiton (Hessian = J'J)
     * @param name solver name
     * @param qp_solver name internally used with casadi::conic (check casadi documetation for conic)
     * @param f RESIDUAL of cost function
     * @param g constraints
     * @param x variables
     * @param opts options for SQPGaussNewton and internal conic (check casadi documetation for conic).
     * NOTE: options for SQPGaussNewton are:
     *                                          "max_iter": iterations used to find solution
     *                                          "reinitialize_qpsolver": if true the internal qp solver is initialized EVERY iteration
     */
    SQPGaussNewton(const std::string& name, const std::string& qp_solver,
                   const CASADI_TYPE& f, const CASADI_TYPE& g, const CASADI_TYPE& x, const casadi::Dict& opts = casadi::Dict()):
        _name(name), _qp_solver(qp_solver),
        _max_iter(1000),
        _reinitialize_qp_solver(false),
        _opts(opts), _qp_opts(opts),
        _alpha(1.)
    {
        _f = casadi::Function("f", {x}, {f}, {"x"}, {"f"});
        _df = _f.function().factory("df", {"x"}, {"jac:f:x"});


        _g = casadi::Function("g",{x}, {g}, {"x"}, {"g"});
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

    void printConicOptions(std::ostream &stream=casadi::uout()) const
    {
        if(_conic)
            _conic->print_options(stream);
    }

    /**
     * @brief setAlpha
     * @param alpha Newton's method step
     */
    void setAlpha(const double alpha)
    {
        _alpha = alpha;
    }

    const double& getAlpha() const
    {
        return _alpha;
    }

    /**
     * @brief solve NLP for given max iteration. Internal qp solver is reinitialized if "reinitialize_qpsolver" option was passed
     * as true in constructor options
     * @param initial_guess_x initial guess
     * @param lbx lower variables bound
     * @param ubx upper variables bound
     * @param lbg lower constraints bound
     * @param ubg upper constraints bound
     * @return solution dictionary containing: "x" solution, "f" 0.5*norm2 cost function, "g" norm2 constraints vector
     */
    const casadi::DMDict& solve(const casadi::DM& initial_guess_x, const casadi::DM& lbx, const casadi::DM& ubx,
                                const casadi::DM& lbg, const casadi::DM& ubg)
    {
        bool sparse = true;
        x0_ = initial_guess_x;
        casadi_utils::toEigen(x0_, _sol);
        _variable_trj[0] = x0_;
        for(unsigned int k = 0; k < _max_iter; ++k)
        {
            //1. Cost function is linearized around actual x0
            _f.setInput(0, _sol); // cost function
            _f.call();

            _df.setInput(0, _sol); // cost function Jacobian
            _df.call(sparse);
            _J = _df.getSparseOutput(0);

            //2. Constraints are linearized around actual x0
            _g_dict.input[_g.name_in(0)] = x0_;
            _g.call(_g_dict.input, _g_dict.output);

            _A_dict.input[_g.name_in(0)] = x0_;
            _dg.call(_A_dict.input, _A_dict.output);

            g_ = _g_dict.output[_g.name_out(0)];
            A_ = _A_dict.output[_dg.name_out(0)];

            //2. We compute Gauss-Newton Hessian approximation and gradient function
            _H.resize(_J.cols(), _J.cols());
            _H = _J.transpose()*_J; ///TODO: to optimize

            _grad = _J.transpose()*_f.getOutput(0);

            //3. Setup QP
            casadi_utils::toCasadiMatrix(_grad, grad_);

            ///TODO: Optimize using directly sparsity
            casadi_utils::toCasadiMatrix(_H.toDense(), H_);

            if(!_conic || _reinitialize_qp_solver)
            {
                _conic_init_input["h"] = H_.sparsity();
                _conic_init_input["a"] = A_.sparsity();
                _conic = std::make_unique<casadi::Function>(casadi::conic("qp_solver", _qp_solver, _conic_init_input, _qp_opts));
            }

            _conic_dict.input["h"] = H_;
            _conic_dict.input["g"] = grad_;
            _conic_dict.input["a"] = A_;
            _conic_dict.input["lba"] = lbg - g_;
            _conic_dict.input["uba"] = ubg - g_;
            _conic_dict.input["lbx"] = lbx - x0_;
            _conic_dict.input["ubx"] = ubx - x0_;
            _conic_dict.input["x0"] = x0_;

            _conic->call(_conic_dict.input, _conic_dict.output);

            //4. Take full step
            x0_ = x0_ + _alpha*_conic_dict.output["x"];
            casadi_utils::toEigen(x0_, _sol);


            // store trajectory
            _variable_trj[k+1] = x0_;
        }

        _solution["x"] = x0_;
        double norm_f = _f.getOutput(0).norm();
        _solution["f"] = 0.5*norm_f*norm_f;
        _solution["g"] = casadi::norm_2(_g_dict.output[_g.name_out(0)].get_elements());
        return _solution;
    }

    /**
     * @brief getVariableTrajectory
     * @return vector of variable solutions (one per iteration)
     */
    const casadi::DMVector& getVariableTrajectory() const
    {
        return _variable_trj;
    }

    /**
     * @brief getObjectiveIterations
     * @return 0.5*norm2 of objective (one per iteration)
     */
    const std::vector<double>& getObjectiveIterations()
    {
        Eigen::VectorXd tmp;
        _objective.clear();
        _objective.reserve(_variable_trj.size());
        for(unsigned int k = 0; k < _variable_trj.size(); ++k)
        {
            casadi_utils::toEigen(_variable_trj[k], tmp);
            _f.setInput(0, tmp); // cost function
            _f.call();
            double norm = _f.getOutput(0).norm();
            _objective.push_back(0.5*norm*norm);
        }
        return _objective;
    }

    /**
     * @brief getConstraintNormIterations
     * @return norm2 of the constraint vector (one per iteration)
     */
    const std::vector<double>& getConstraintNormIterations()
    {
        _constraints_norm.clear();
        _constraints_norm.reserve(_variable_trj.size());
        for(auto sol : _variable_trj)
        {
            _g_dict.input[_g.name_in(0)] = sol;
            _g.call(_g_dict.input, _g_dict.output);
            _constraints_norm.push_back(casadi::norm_2(_g_dict.output[_g.name_out(0)].get_elements()));
        }
        return _constraints_norm;
    }



private:


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
    std::vector<double> _objective, _constraints_norm;

    Eigen::SparseMatrix<double> _J;
    Eigen::SparseMatrix<double> _H;
    Eigen::VectorXd _grad;
    casadi::DM grad_;
    casadi::DM g_;
    casadi::DM A_;
    casadi::DM H_;
    casadi::DM x0_;
    Eigen::VectorXd _sol;

    IODMDict _g_dict;
    IODMDict _A_dict;

    double _alpha;

};

}

#endif
