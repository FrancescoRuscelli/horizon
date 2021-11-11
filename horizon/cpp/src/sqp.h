#ifndef __HORIZON__SQP__H__
#define __HORIZON__SQP__H__

#include <casadi/casadi.hpp>
#include "wrapped_function.h"
#include <Eigen/Dense>
#include <memory>
#include <chrono>


#include "profiling.h"

#include "ilqr.h"

namespace horizon{

typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;
typedef Eigen::Ref<const Eigen::MatrixXd> MatConstRef;

template <class CASADI_TYPE> ///casadi::SX or casadi::MX
class SQPGaussNewton
{
public:

    static void setQPOasesOptionsMPC(casadi::Dict& opts)
    {
        opts["enableEqualities"] = true;
        opts["initialStatusBounds"] = "inactive";
        opts["numRefinementSteps"] = 0;
        opts["enableDriftCorrection"] = 0;
        opts["terminationTolerance"] = 10e9 * std::numeric_limits<double>::epsilon();
        opts["enableFlippingBounds"] = false;
        opts["enableNZCTests"] = false;
        opts["enableRamping"] = false;
        opts["enableRegularisation"] = true;
        opts["numRegularisationSteps"] = 2;
        opts["epsRegularisation"] = 5. * 10e3 * std::numeric_limits<double>::epsilon();
    }

    static void setQPOasesOptionsReliable(casadi::Dict& opts)
    {
        opts["enableEqualities"] = false;
        opts["numRefinementSteps"] = 2;
        opts["enableFullLITest"] = true;
        opts["epsLITests"] = 10e5 * std::numeric_limits<double>::epsilon();
        opts["maxDualJump"] = 10e8;
        opts["enableCholeskyRefactorisation"] = 1;
    }


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
     *          "max_iter": iterations used to find solution
     *          "reinitialize_qpsolver": if true the internal qp solver is initialized EVERY iteration
     *          "solution_convergence": iteration are stoped if solution does not change under this threshold
     */
    SQPGaussNewton(const std::string& name, const std::string& qp_solver,
                   const casadi::Function& f, const casadi::Function& g, const casadi::Dict& opts = casadi::Dict()):
        _name(name), _qp_solver(qp_solver),
        _max_iter(1000),
        _reinitialize_qp_solver(false),
        _opts(opts), _qp_opts(opts),
        _alpha(1.), _solution_convergence(1e-6),_alpha_min(1e-3),
        _fpr(0, 0, 0) ///TODO: this needs to be improved!
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

        if(opts.count("max_iter"))
        {
            _max_iter = opts.at("max_iter");
            _qp_opts.erase("max_iter");
        }

        if(opts.count("reinitialize_qpsolver"))
        {
            _reinitialize_qp_solver = opts.at("reinitialize_qpsolver");
            _qp_opts.erase("reinitialize_qpsolver");
        }

        if(opts.count("solution_convergence"))
        {
            _solution_convergence = opts.at("solution_convergence");
            _qp_opts.erase("solution_convergence");
        }


        _variable_trj.resize(_max_iter+1, casadi::DM(f.size1_in(0), f.size2_in(0)));

        _hessian_computation_time.reserve(_max_iter);
        _qp_computation_time.reserve(_max_iter);
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
        _alpha(1.), _solution_convergence(1e-6), _alpha_min(1e-3),
        _fpr(0, 0, 0) ///TODO: this needs to be improved!
    {
        _f = casadi::Function("f", {x}, {f}, {"x"}, {"f"});
        _df = _f.function().factory("df", {"x"}, {"jac:f:x"});


        _g = casadi::Function("g",{x}, {g}, {"x"}, {"g"});
        _dg = _g.factory("dg", {"x"}, {"jac:g:x"});

        if(opts.count("max_iter"))
        {
            _max_iter = opts.at("max_iter");
            _qp_opts.erase("max_iter");
        }

        if(opts.count("reinitialize_qpsolver"))
        {
            _reinitialize_qp_solver = opts.at("reinitialize_qpsolver");
            _qp_opts.erase("reinitialize_qpsolver");
        }

        if(opts.count("solution_convergence"))
        {
            _solution_convergence = opts.at("solution_convergence");
            _qp_opts.erase("solution_convergence");
        }

        _variable_trj.resize(_max_iter+1, casadi::DM(x.rows(), x.columns()));

        _hessian_computation_time.reserve(_max_iter);
        _qp_computation_time.reserve(_max_iter);

    }

    void printConicOptions(std::ostream &stream=casadi::uout()) const
    {
        if(_conic)
            _conic->print_options(stream);
    }

    /**
     * @brief setAlphaMin set the minumi allowed alpha during linesearch
     * @param alpha min in Newton's method step
     */
    void setAlphaMin(const double alpha_min)
    {
        _alpha_min = alpha_min;
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
     * @param p parameters (NOT USED AT THE MOMENT!)
     * @return solution dictionary containing: "x" solution, "f" 0.5*norm2 cost function, "g" norm2 constraints vector
     */
    const casadi::DMDict& solve(const casadi::DM& initial_guess_x, const casadi::DM& lbx, const casadi::DM& ubx,
                                const casadi::DM& lbg, const casadi::DM& ubg, const casadi::DM& p = casadi::DM())
    {
        _hessian_computation_time.clear();
        _qp_computation_time.clear();

        bool sparse = true;
        x0_ = initial_guess_x;
        casadi_utils::toEigen(x0_, _sol);
        _variable_trj[0] = x0_;
        _iteration_to_solve = 0;
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
            auto tic = std::chrono::high_resolution_clock::now();
            _H.resize(_J.cols(), _J.cols());
            //_H.selfadjointView<Eigen::Lower>().rankUpdate(_J.transpose());
            _H = _J.transpose() * _J;
            auto toc = std::chrono::high_resolution_clock::now();
            _hessian_computation_time.push_back((toc-tic).count()*1E-9);

            _grad = _J.transpose()*_f.getOutput(0);

            //3. Setup QP
            casadi_utils::toCasadiMatrix(_grad, grad_);

            if(!H_.is_init())
                H_ = casadi_utils::WrappedSparseMatrix<double>(_H);
            else
                H_.update_values(_H);

            if(!_conic || _reinitialize_qp_solver)
            {
                _conic_init_input["h"] = H_.get().sparsity();
                _conic_init_input["a"] = A_.sparsity();
                _conic = std::make_unique<casadi::Function>(casadi::conic("qp_solver", _qp_solver, _conic_init_input, _qp_opts));
            }

            _conic_dict.input["h"] = H_.get();
            _conic_dict.input["g"] = grad_;
            _conic_dict.input["a"] = A_;
            _conic_dict.input["lba"] = lbg - g_;
            _conic_dict.input["uba"] = ubg - g_;
            _conic_dict.input["lbx"] = lbx - x0_;
            _conic_dict.input["ubx"] = ubx - x0_;
            _conic_dict.input["x0"] = x0_;

            tic = std::chrono::high_resolution_clock::now();
            _conic->call(_conic_dict.input, _conic_dict.output);
            toc = std::chrono::high_resolution_clock::now();
            _qp_computation_time.push_back((toc-tic).count()*1E-9);

            casadi_utils::toEigen(_conic_dict.output["x"], _dx);
            casadi_utils::toEigen(_conic_dict.output["lam_a"], _lam_a);
            casadi_utils::toEigen(_conic_dict.output["lam_x"], _lam_x);

            /// BREAK CRITERIA
            if(_dx.norm() <= _solution_convergence)
                break;


            casadi_utils::toEigen(x0_, _sol);
            Eigen::VectorXd dx;
            casadi_utils::toEigen(_conic_dict.output["x"], dx);
            if(lineSearch(_sol, dx, _lam_a, _lam_x, lbg, ubg, lbx, ubx))
            {
                casadi_utils::toCasadiMatrix(_sol, x0_);
                // store trajectory
                _variable_trj[k+1] = x0_;

                _iteration_to_solve++;
                _fpr.iter = _iteration_to_solve;
            }
            else
                throw std::runtime_error("Linesearch failed, unable to solve");

            _iter_cb(_fpr);

        }

        _solution["x"] = x0_;
        double norm_f = _f.getOutput(0).norm();
        _solution["f"] = 0.5*norm_f*norm_f;
        _solution["g"] = casadi::norm_2(_g_dict.output[_g.name_out(0)].get_elements());

        return _solution;
    }

    double getCost(const Eigen::VectorXd& x)
    {
        _f.setInput(0, x);
        _f.call();
        return _f.getOutput(0).squaredNorm();
    }

    double getCostDerivative(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)
    {
        _df.setInput(0, x); // cost function Jacobian
        _df.call(true);
        _J = _df.getSparseOutput(0);
        Eigen::VectorXd grad = _J.transpose()*_f.getOutput(0);
        return dx.dot(grad);
    }

    double getConstraintViolation(const Eigen::VectorXd& x,
                                  const Eigen::VectorXd& lbg, const Eigen::VectorXd& ubg,
                                  const Eigen::VectorXd& lbx, const Eigen::VectorXd& ubx)
    {
        casadi::DM _x_;
        casadi_utils::toCasadiMatrix(x, _x_);
        _g_dict.input[_g.name_in(0)] = _x_;
        _g.call(_g_dict.input, _g_dict.output);
        Eigen::VectorXd g;
        casadi_utils::toEigen(_g_dict.output[_g.name_out(0)], g);


        return (lbg-g).cwiseMax(0.).lpNorm<1>() + (ubg-g).cwiseMin(0.).lpNorm<1>() +
               (lbx-x).cwiseMax(0.).lpNorm<1>() + (ubx-x).cwiseMin(0.).lpNorm<1>();
    }

    bool lineSearch(Eigen::VectorXd& x, const Eigen::VectorXd& dx, const Eigen::VectorXd& lam_x, const Eigen::VectorXd& lam_a,
                    const casadi::DM& lbg, const casadi::DM& ubg,
                    const casadi::DM& lbx, const casadi::DM& ubx)
    {
        Eigen::VectorXd _lbg_, _ubg_, _lbx_, _ubx_;
        casadi_utils::toEigen(lbg, _lbg_);
        casadi_utils::toEigen(ubg, _ubg_);
        casadi_utils::toEigen(lbx, _lbx_);
        casadi_utils::toEigen(ubx, _ubx_);

        Eigen::VectorXd x0 = x;


        const double merit_safety_factor = 2.0;
        double norminf_lam_x = lam_x.lpNorm<Eigen::Infinity>();
        double norminf_lam_a = lam_a.lpNorm<Eigen::Infinity>();
        double norminf_lam = merit_safety_factor*std::max(norminf_lam_x, norminf_lam_a);

        double initial_cost = getCost(x0);

        double cost_derr = getCostDerivative(x0, dx);

        double constraint_violation = getConstraintViolation(x0, _lbg_, _ubg_, _lbx_, _ubx_);

        double merit_der = cost_derr - norminf_lam * constraint_violation;
        double initial_merit = initial_cost + norminf_lam*constraint_violation;

        _alpha = 1.;
        bool accepted = false;
        double eta = 1e-4;
        while( _alpha > _alpha_min)
        {
            x = x0 + _alpha*dx;
            double candidate_cost = getCost(x);
            double candidate_constraint_violation = getConstraintViolation(x, _lbg_, _ubg_, _lbx_, _ubx_);

            double candidate_merit = candidate_cost + norminf_lam*candidate_constraint_violation;

            // evaluate Armijo's condition
            accepted = candidate_merit <= initial_merit + eta*_alpha*merit_der;

            _fpr.alpha = _alpha;
            _fpr.cost = candidate_cost;
            _fpr.constraint_violation = candidate_constraint_violation;
            _fpr.merit = candidate_merit;
            _fpr.step_length = _alpha * dx.norm();
            _fpr.accepted = accepted;

            if(accepted)
                break;

            _alpha *= 0.5;
        }

        if(!accepted)
            return false;
        return true;
    }

    void f(const CASADI_TYPE& f, const CASADI_TYPE& x, bool reinitialize_qp_solver = true)
    {
        _reinitialize_qp_solver = reinitialize_qp_solver;

        _f = casadi::Function("f", {x}, {f}, {"x"}, {"f"});
        _df = _f.function().factory("df", {"x"}, {"jac:f:x"});
    }

    void g(const CASADI_TYPE& g, const CASADI_TYPE& x, bool reinitialize_qp_solver = true)
    {
        _reinitialize_qp_solver = reinitialize_qp_solver;

        _g = casadi::Function("g",{x}, {g}, {"x"}, {"g"});
        _dg = _g.factory("dg", {"x"}, {"jac:g:x"});
    }

    bool f(const casadi::Function& f, bool reinitialize_qp_solver = true)
    {
        _reinitialize_qp_solver = reinitialize_qp_solver;

        if(f.n_in() != 1)
            return false;
        if(f.n_out() != 1)
            return false;

        _f = f;
        _df = f.factory("df", {f.name_in(0)}, {"jac:" + f.name_out(0) +":" + f.name_in(0)});

        return true;
    }

    bool g(const casadi::Function& g, bool reinitialize_qp_solver = true)
    {
        _reinitialize_qp_solver = reinitialize_qp_solver;

        if(g.n_in() != 1)
            return false;
        if(g.n_out() != 1)
            return false;

        _g = g;
        _dg = _g.factory("dg", {g.name_in(0)}, {"jac:" + g.name_out(0) + ":" + g.name_in(0)});

        return true;
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
     * @brief getNumberOfIterations
     * @return number of iteration to solve NLP
     */
    unsigned int getNumberOfIterations()
    {
        return _iteration_to_solve;
    }

    /**
     * @brief getObjectiveIterations
     * @return 0.5*norm2 of objective (one per iteration)
     */
    const std::vector<double>& getObjectiveIterations()
    {
        Eigen::VectorXd tmp;
        _objective.clear();
        _objective.reserve(_iteration_to_solve);
        for(unsigned int k = 0; k < _iteration_to_solve; ++k)
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
        _constraints_norm.reserve(_iteration_to_solve);
        for(unsigned int k = 0; k < _iteration_to_solve; ++k)
        {
            _g_dict.input[_g.name_in(0)] = _variable_trj[k];
            _g.call(_g_dict.input, _g_dict.output);
            _constraints_norm.push_back(casadi::norm_2(_g_dict.output[_g.name_out(0)].get_elements()));
        }
        return _constraints_norm;
    }

    /**
     * @brief getHessianComputationTime
     * @return vector of times needed to compute hessian (one value per iteration)
     */
    const std::vector<double>& getHessianComputationTime() const
    {
        return _hessian_computation_time;
    }

    /**
     * @brief getQPComputationTime
     * @return vector of times needed to solve qp (one value per iteration)
     */
    const std::vector<double>& getQPComputationTime() const
    {
        return _qp_computation_time;
    }

    typedef std::function<bool(const IterativeLQR::ForwardPassResult& res)> CallbackType;
    void setIterationCallback(const CallbackType& cb)
    {
        _iter_cb = cb;
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
    casadi_utils::WrappedSparseMatrix<double> H_;
    casadi::DM x0_;
    Eigen::VectorXd _sol, _dx, _lam_a, _lam_x;

    IODMDict _g_dict;
    IODMDict _A_dict;

    double _alpha, _alpha_min;

    std::vector<double> _hessian_computation_time;
    std::vector<double> _qp_computation_time;

    double _solution_convergence;

    unsigned int _iteration_to_solve;

    IterativeLQR::ForwardPassResult _fpr;
    CallbackType _iter_cb;

};

}

#endif
