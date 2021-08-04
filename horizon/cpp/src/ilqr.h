#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <memory>

#include "wrapped_function.h"

namespace horizon
{

typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;

class IterativeLQR
{

public:

    IterativeLQR(casadi::Function fdyn,
                 int N);

    void setIntermediateCost(const std::vector<casadi::Function>& inter_cost);

    void setFinalCost(const casadi::Function& final_cost);

    void setFinalConstraint(const casadi::Function& final_constraint);

    void setInitialState(const Eigen::VectorXd& x0);

    bool solve(int max_iter);

    const Eigen::MatrixXd& getStateTrajectory() const;
    const Eigen::MatrixXd& getInputTrajectory() const;

    // all public to test things
    void linearize_quadratize();
    void backward_pass();
    bool forward_pass(double alpha);
    VecConstRef state(int i) const;
    VecConstRef input(int i) const;

    ~IterativeLQR();

protected:

private:

    struct ConstrainedDynamics;
    struct ConstrainedCost;
    typedef std::tuple<int, ConstrainedDynamics, ConstrainedCost> HandleConstraintsRetType;
    struct Dynamics;
    struct Constraint;
    struct IntermediateCost;
    struct Temporaries;

    void backward_pass_iter(int i);
    HandleConstraintsRetType handle_constraints(int i);
    void forward_pass_iter(int i, double alpha);
    void set_default_cost();

    

    struct ConstraintToGo
    {
        ConstraintToGo(int nx);

        void set(Eigen::Ref<const Eigen::MatrixXd> C,
                 Eigen::Ref<const Eigen::VectorXd> h);

        void set(const Constraint& constr);

        void clear();

        int dim() const;

        Eigen::Ref<const Eigen::MatrixXd> C() const;

        VecConstRef h() const;

    private:

        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> _C;
        Eigen::VectorXd _h;
        int _dim;
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
