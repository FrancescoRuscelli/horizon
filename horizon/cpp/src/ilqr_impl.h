#ifndef ILQR_IMPL_H
#define ILQR_IMPL_H

#include "ilqr.h"
#include "wrapped_function.h"

using namespace horizon;
using namespace casadi_utils;

namespace cs = casadi;

extern utils::Timer::TocCallback on_timer_toc;

struct IterativeLQR::Dynamics
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
    Eigen::VectorXd d;

    Dynamics(int nx, int nu);

    VecConstRef integrate(VecConstRef x, VecConstRef u);

    void linearize(VecConstRef x, VecConstRef u);

    void computeDefect(VecConstRef x, VecConstRef u, VecConstRef xnext, Eigen::VectorXd& d);

    void setDynamics(casadi::Function f);

};

struct IterativeLQR::Constraint
{
    // constraint function
    casadi_utils::WrappedFunction f;

    // constraint jacobian
    casadi_utils::WrappedFunction df;

    // dh/dx
    const Eigen::MatrixXd& C() const;

    // dh/du
    const Eigen::MatrixXd& D() const;

    // constraint violation f(x, u) - hd
    VecConstRef h() const;

    // desired value
    Eigen::VectorXd hd;

    // valid flag
    bool is_valid() const;

    Constraint();

    void linearize(VecConstRef x, VecConstRef u);

    void evaluate(VecConstRef x, VecConstRef u);

    void setConstraint(casadi::Function h);

};

struct IterativeLQR::IntermediateCost
{
    // original cost
    casadi_utils::WrappedFunction l;

    // cost gradient
    casadi_utils::WrappedFunction dl;

    // cost hessian
    casadi_utils::WrappedFunction ddl;

    /* Quadratized cost */
    const Eigen::MatrixXd& Q() const;
    VecConstRef q() const;
    const Eigen::MatrixXd& R() const;
    VecConstRef r() const;
    const Eigen::MatrixXd& P() const;

    IntermediateCost(int nx, int nu);

    void setCost(const casadi::Function& cost);

    double evaluate(VecConstRef x, VecConstRef u);
    void quadratize(VecConstRef x, VecConstRef u);
};

struct IterativeLQR::Temporaries
{
    /* Backward pass */

    // temporary for s + S*d
    Eigen::MatrixXd s_plus_S_d;

    // temporary for S*A
    Eigen::MatrixXd S_A;

    // quadratized value function (after constraints, if any)
    // note: 'u' is actually 'z' in this context!
    Eigen::MatrixXd Huu;
    Eigen::MatrixXd Hux;
    Eigen::MatrixXd Hxx;

    Eigen::VectorXd hx;
    Eigen::VectorXd hu;

    // quadratized value function (free)
    Eigen::MatrixXd Huuf;
    Eigen::MatrixXd Huxf;
    Eigen::VectorXd huf;

    // temporary for [hu Hux]
    // note: it is used to store gains as well, take care!
    Eigen::MatrixXd huHux;

    // cholesky for Huu
    Eigen::LLT<Eigen::MatrixXd> llt;

    // linearized constraint to go (C*x + D*u + h = 0)
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;
    Eigen::VectorXd h;

    // svd of D
    Eigen::BDCSVD<Eigen::MatrixXd> svd;

    // rotated constraint according to left singular vectors of D
    Eigen::MatrixXd rotC;
    Eigen::VectorXd roth;

    // temporary that is used to compute lagrange multipliers
    Eigen::MatrixXd UrSinvVrT;

    // input component due to constraint (u = lc + Lc*x + Bz*z)
    Eigen::MatrixXd Lc;
    Eigen::VectorXd lc;
    Eigen::MatrixXd Bz;

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


    /* Forward pass */
    Eigen::VectorXd dx;
    Eigen::VectorXd defect;
    Eigen::VectorXd lam_x;
    Eigen::VectorXd lam_g;

};

struct IterativeLQR::ConstraintToGo
{
    ConstraintToGo(int nx, int nu);

    void set(MatConstRef C, VecConstRef h);

    void set(const Constraint& constr);

    void propagate_backwards(MatConstRef A, MatConstRef B, VecConstRef d);

    void add(const Constraint& constr);

    void clear();

    int dim() const;

    MatConstRef C() const;

    MatConstRef D() const;

    VecConstRef h() const;

private:

    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> _C;
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> _D;
    Eigen::VectorXd _h;
    int _dim;
};

struct IterativeLQR::ValueFunction
{
    Eigen::MatrixXd S;
    Eigen::VectorXd s;

    ValueFunction(int nx);
};

struct IterativeLQR::BackwardPassResult
{
    // real input as function of state
    // (u = Lu*x + lu)
    Eigen::MatrixXd Lu;
    Eigen::VectorXd lu;

    // auxiliary input as function of state
    // (z = Lz*x + lz, where u = lc + Lc*x + Bz*z)
    Eigen::MatrixXd Lz;
    Eigen::VectorXd lz;

    // constraint-to-go size
    int nc;

    // lagrange multipliers
    Eigen::MatrixXd Gu;
    Eigen::MatrixXd Gx;
    Eigen::VectorXd glam;

    BackwardPassResult(int nx, int nu);
};

struct IterativeLQR::ConstrainedDynamics
{
    MatConstRef A;
    MatConstRef B;
    VecConstRef d;
};

struct IterativeLQR::ConstrainedCost
{
    MatConstRef Q;
    MatConstRef R;
    MatConstRef P;
    VecConstRef q;
    VecConstRef r;
};

#endif // ILQR_IMPL_H