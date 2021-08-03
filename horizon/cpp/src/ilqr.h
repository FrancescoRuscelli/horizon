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

    void setFinalConstraint(const casadi::Function& final_constraint);

    void setInitialState(const Eigen::VectorXd& x0);

    const Eigen::MatrixXd& getStateTrajectory() const;
    const Eigen::MatrixXd& getInputTrajectory() const;

    // all public to test things
    void linearize_quadratize();
    void backward_pass();
    bool forward_pass(double alpha);
    Eigen::Ref<Eigen::VectorXd> state(int i);
    Eigen::Ref<Eigen::VectorXd> input(int i);

protected:

private:

    struct ConstrainedDynamics;
    struct ConstrainedCost;
    typedef std::tuple<int, IterativeLQR::ConstrainedDynamics, IterativeLQR::ConstrainedCost> HandleConstraintsRetType;

    void backward_pass_iter(int i);
    HandleConstraintsRetType handle_constraints(int i);
    void forward_pass_iter(int i, double alpha);
    void set_default_cost();

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

        Eigen::Ref<const Eigen::VectorXd> integrate(const Eigen::VectorXd& x,
                                                    const Eigen::VectorXd& u);

        void linearize(const Eigen::VectorXd& x, const Eigen::VectorXd& u);

        void computeDefect(const Eigen::VectorXd& x,
                           const Eigen::VectorXd& u,
                           const Eigen::VectorXd& xnext);

        void setDynamics(casadi::Function f);

    };

    struct Constraint
    {
        // constraint function
        casadi_utils::WrappedFunction f;

        // constraint jacobian
        casadi_utils::WrappedFunction df;

        // dh/dx
        const Eigen::MatrixXd& C() const;

        // dh/du
        const Eigen::MatrixXd& D() const;

        // constraint value
        Eigen::Ref<const Eigen::VectorXd> h() const;

        // valid flag
        bool is_valid() const;

        Constraint();

        void linearize(const Eigen::VectorXd& x, const Eigen::VectorXd& u);

        void setConstraint(casadi::Function h);

    };

    struct ConstraintToGo
    {
        ConstraintToGo(int nx);

        void set(Eigen::Ref<const Eigen::MatrixXd> C,
                 Eigen::Ref<const Eigen::VectorXd> h);

        void set(const Constraint& constr);

        void clear();

        int dim() const;

        Eigen::Ref<const Eigen::MatrixXd> C() const;

        Eigen::Ref<const Eigen::VectorXd> h() const;

    private:

        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> _C;
        Eigen::VectorXd _h;
        int _dim;
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
        const Eigen::MatrixXd& Q() const;
        Eigen::Ref<const Eigen::VectorXd> q() const;
        const Eigen::MatrixXd& R() const;
        Eigen::Ref<const Eigen::VectorXd> r() const;
        const Eigen::MatrixXd& P() const;

        IntermediateCost(int nx, int nu);

        void setCost(const casadi::Function& cost);

        void quadratize(const Eigen::VectorXd& x, const Eigen::VectorXd& u);
    };

    struct ConstrainedDynamics
    {
        Eigen::Ref<const Eigen::MatrixXd> A;
        Eigen::Ref<const Eigen::MatrixXd> B;
        Eigen::Ref<const Eigen::VectorXd> d;
    };

    struct ConstrainedCost
    {
        Eigen::Ref<const Eigen::MatrixXd> Q;
        Eigen::Ref<const Eigen::MatrixXd> R;
        Eigen::Ref<const Eigen::MatrixXd> P;
        Eigen::Ref<const Eigen::VectorXd> q;
        Eigen::Ref<const Eigen::VectorXd> r;
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
        // backward pass
        Eigen::MatrixXd s_plus_S_d;
        Eigen::MatrixXd S_A;

        Eigen::MatrixXd Huu;
        Eigen::MatrixXd Hux;
        Eigen::MatrixXd Hxx;

        Eigen::VectorXd hx;
        Eigen::VectorXd hu;

        Eigen::MatrixXd huHux;

        Eigen::LLT<Eigen::MatrixXd> llt;

        // constraints
        Eigen::MatrixXd C;
        Eigen::MatrixXd D;
        Eigen::VectorXd h;
        Eigen::MatrixXd rotC;
        Eigen::VectorXd roth;
        Eigen::BDCSVD<Eigen::MatrixXd> svd;
        Eigen::MatrixXd Lc;
        Eigen::MatrixXd Lz;
        Eigen::VectorXd lc;

        // modified dynamics and cost due to
        // constraints
        Eigen::MatrixXd Ac;
        Eigen::MatrixXd Bc;
        Eigen::VectorXd dc;
        Eigen::MatrixXd Qc;
        Eigen::MatrixXd Rc;
        Eigen::MatrixXd Pc;
        Eigen::VectorXd qc;
        Eigen::VectorXd rc;

        // forward pass
        Eigen::VectorXd dx;

    };

    int _nx;
    int _nu;
    int _N;

    casadi::Function _f;
    casadi::Function _df;

    std::vector<IntermediateCost> _cost;
    std::vector<Constraint> _constraint;
    std::vector<ValueFunction> _value;
    std::vector<Dynamics> _dyn;

    std::vector<BackwardPassResult> _bp_res;
    ConstraintToGo _constraint_to_go;
    ForwardPassResult _fp_res;

    Eigen::MatrixXd _xtrj;
    Eigen::MatrixXd _utrj;

    std::vector<Temporaries> _tmp;
};



}
