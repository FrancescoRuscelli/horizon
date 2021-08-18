#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <memory>

#include "profiling.h"

namespace horizon
{

typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;
typedef Eigen::Ref<const Eigen::MatrixXd> MatConstRef;


/**
 * @brief IterativeLQR implements a multiple-shooting variant of the
 * notorious ILQR algorithm, implemented following the paper
 * "A Family of Iterative Gauss-Newton Shooting Methods for Nonlinear
 * Optimal Control" by M. Giftthaler, et al., from which most of the notation
 * is taken.
 *
 * The framework supports arbitrary (differentiable) discrete time dynamics
 * systems as well as arbitrary (twice differentiable) cost functions.
 *
 * Furthermore, arbitrary (differentiable) equality constraints are treated
 * with a projection approach.
 */
class IterativeLQR
{

public:

    /**
     *
     */
    struct ForwardPassResult;

    /**
     * @brief CallbackType
     */
    typedef std::function<bool(const ForwardPassResult& res)> CallbackType;


    /**
     * @brief Class constructor
     * @param fdyn is a function mapping state and control to the integrated state;
     * required signature is (x, u) -> (f)
     * @param N is the number of shooting intervals
     */
    IterativeLQR(casadi::Function fdyn,
                 int N);

    /**
     * @brief setStepLength
     * @param alpha
     */
    void setStepLength(double alpha);

    /**
     * @brief set an intermediate cost term for each intermediate state
     * @param inter_cost: a vector of N entries, each of which is a function with
     * required signature (x, u) -> (l)
     */
    void setIntermediateCost(const std::vector<casadi::Function>& inter_cost);

    /**
     * @brief set an intermediate cost term for the k-th intermediate state
     * @param k: the node that the cost refers to
     * @param inter_cost: a function with required signature (x, u) -> (l)
     */
    void setIntermediateCost(int k, const casadi::Function& inter_cost);

    /**
     * @brief set the final cost
     * @param final_cost: a function with required signature (x, u) -> (l),
     * even though the input 'u' is not used
     */
    void setFinalCost(const casadi::Function& final_cost);

    /**
     * @brief  set an intermediate constraint term for the k-th intermediate state
     * @param k: the node that the cost refers to
     * @param inter_constraint: a function with required signature (x, u) -> (h),
     * where the constraint is h(x, u) = 0
     */
    void setIntermediateConstraint(int k, const casadi::Function& inter_constraint);

    void setIntermediateConstraint(const std::vector<casadi::Function>& inter_constraint);

    void setFinalConstraint(const casadi::Function& final_constraint);

    void setInitialState(const Eigen::VectorXd& x0);

    void setStateInitialGuess(const Eigen::MatrixXd& x0);

    void setInputInitialGuess(const Eigen::MatrixXd& u0);

    void setIterationCallback(const CallbackType& cb);

    bool solve(int max_iter);

    const Eigen::MatrixXd& getStateTrajectory() const;

    const Eigen::MatrixXd& getInputTrajectory() const;

    const utils::ProfilingInfo& getProfilingInfo() const;

    VecConstRef state(int i) const;

    VecConstRef input(int i) const;

    ~IterativeLQR();

    struct ForwardPassResult
    {
        Eigen::MatrixXd xtrj;
        Eigen::MatrixXd utrj;
        double alpha;
        double cost;
        double merit;
        double mu_f;
        double mu_c;
        double merit_der;
        double step_length;
        double constraint_violation;
        double defect_norm;
        int iter;
        bool accepted;

        ForwardPassResult(int nx, int nu, int N);
    };



protected:

private:

    struct ConstrainedDynamics;
    struct ConstrainedCost;
    struct Dynamics;
    struct Constraint;
    struct IntermediateCost;
    struct Temporaries;
    struct ConstraintToGo;
    struct BackwardPassResult;
    struct ValueFunction;

    typedef std::tuple<int, ConstrainedDynamics, ConstrainedCost> HandleConstraintsRetType;

    void linearize_quadratize();
    void report_result(const ForwardPassResult& fpres);
    void backward_pass();
    void backward_pass_iter(int i);
    HandleConstraintsRetType handle_constraints(int i);
    double compute_merit_value(double mu_f, double mu_c, double cost, double defect_norm, double constr_viol);
    double compute_merit_slope(double mu_f, double mu_c, double defect_norm, double constr_viol);
    std::pair<double, double> compute_merit_weights();
    double compute_cost(const Eigen::MatrixXd& xtrj, const Eigen::MatrixXd& utrj);
    double compute_constr(const Eigen::MatrixXd& xtrj, const Eigen::MatrixXd& utrj);
    double compute_defect(const Eigen::MatrixXd& xtrj, const Eigen::MatrixXd& utrj);
    bool forward_pass(double alpha);
    void forward_pass_iter(int i, double alpha);
    void line_search(int iter);
    bool should_stop();

    void set_default_cost();

    const int _nx;
    const int _nu;
    const int _N;

    double _step_length;

    std::vector<IntermediateCost> _cost;
    std::vector<Constraint> _constraint;
    std::vector<ValueFunction> _value;
    std::vector<Dynamics> _dyn;

    std::vector<BackwardPassResult> _bp_res;
    std::unique_ptr<ConstraintToGo> _constraint_to_go;
    std::unique_ptr<ForwardPassResult> _fp_res;
    std::unique_ptr<ForwardPassResult> _fp_best;

    Eigen::MatrixXd _xtrj;
    Eigen::MatrixXd _utrj;
    std::vector<Eigen::VectorXd> _lam_g;
    Eigen::MatrixXd _lam_x;


    std::vector<Temporaries> _tmp;

    CallbackType _iter_cb;
    utils::ProfilingInfo _prof_info;
};



}
