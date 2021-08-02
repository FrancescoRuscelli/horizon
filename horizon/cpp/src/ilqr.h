#include <casadi/casadi.hpp>
#include <Eigen/Dense>

#include "wrapped_function.h"

namespace horizon
{

class IterativeLQR
{

public:

    IterativeLQR(casadi::Function fdyn,
                 int N);

    void setIntermediateCost(const std::vector<casadi::Function>& inter_cost);

    void setFinalCost(const casadi::Function& final_cost);

    void setInitialState(const Eigen::VectorXd& x0);

    const Eigen::MatrixXd& getStateTrajectory() const;
    const Eigen::MatrixXd& getInputTrajectory() const;

    // all public to test things
    void linearize_quadratize();
    void compute_defect(int i, Eigen::VectorXd& d);
    void backward_pass();
    bool forward_pass(double alpha);
    void backward_pass_iter(int i);
    void forward_pass_iter(int i, double alpha);
    Eigen::Ref<Eigen::VectorXd> state(int i);
    Eigen::Ref<Eigen::VectorXd> input(int i);

protected:

private:

    struct Dynamics
    {

    public:

        // dynamics function
        casadi_utils::WrappedFunction f;

        // dynamics jacobian
        casadi_utils::WrappedFunction df;

        // df/dx
        const Eigen::MatrixXd& A() const;

        // df/du
        const Eigen::MatrixXd& B() const;

        // defect (or gap)
        // this is not computed by this class, and
        // must be filled from outside
        Eigen::VectorXd d;

        Dynamics(int nx, int nu);

        void linearize(const Eigen::VectorXd& x, const Eigen::VectorXd& u);

    };

    struct IntermediateCost
    {
        // original cost
        casadi_utils::WrappedFunction l;

        // cost gradient
        casadi_utils::WrappedFunction dl;

        // cost hessian
        casadi_utils::WrappedFunction ddl;

        /* Quadratized cost */
        Eigen::Ref<const Eigen::MatrixXd> Q() const;
        Eigen::Ref<const Eigen::VectorXd> q() const;
        Eigen::Ref<const Eigen::MatrixXd> R() const;
        Eigen::Ref<const Eigen::VectorXd> r() const;
        Eigen::Ref<const Eigen::MatrixXd> P() const;

        IntermediateCost(int nx, int nu);

        void setCost(const casadi::Function& cost);

        void quadratize(const Eigen::VectorXd& x, const Eigen::VectorXd& u);
    };

    struct ValueFunction
    {
        Eigen::MatrixXd S;
        Eigen::VectorXd s;

        ValueFunction(int nx);
    };

    struct BackwardPassResult
    {
        Eigen::MatrixXd Lfb;
        Eigen::VectorXd du_ff;

        BackwardPassResult(int nx, int nu);
    };

    struct ForwardPassResult
    {
        Eigen::MatrixXd xtrj;
        Eigen::MatrixXd utrj;

        ForwardPassResult(int nx, int nu, int N);
    };

    struct Temporaries
    {
        Eigen::MatrixXd s_plus_S_d;
        Eigen::MatrixXd S_A;

        Eigen::MatrixXd Huu;
        Eigen::MatrixXd Hux;
        Eigen::MatrixXd Hxx;

        Eigen::VectorXd hx;
        Eigen::VectorXd hu;

        Eigen::MatrixXd huHux;

        Eigen::LLT<Eigen::MatrixXd> llt;

        Eigen::VectorXd dx;
    };

    int _nx;
    int _nu;
    int _N;

    casadi::Function _f;
    casadi::Function _df;

    std::vector<IntermediateCost> _cost;
    std::vector<ValueFunction> _value;
    std::vector<Dynamics> _dyn;

    std::vector<BackwardPassResult> _bp_res;
    ForwardPassResult _fp_res;

    Eigen::MatrixXd _xtrj;
    Eigen::MatrixXd _utrj;

    Temporaries tmp;
};



}
