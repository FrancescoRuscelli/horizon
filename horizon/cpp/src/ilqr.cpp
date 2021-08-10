#include "ilqr.h"
#include "wrapped_function.h"

using namespace horizon;
using namespace casadi_utils;

namespace cs = casadi;

utils::Timer::TocCallback on_timer_toc;

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

    void computeDefect(VecConstRef x, VecConstRef u, VecConstRef xnext);

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

    // constraint value
    VecConstRef h() const;

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
    Eigen::MatrixXd Lfb;
    Eigen::VectorXd du_ff;

    BackwardPassResult(int nx, int nu);
};

struct IterativeLQR::ForwardPassResult
{
    Eigen::MatrixXd xtrj;
    Eigen::MatrixXd utrj;
    double cost;
    double step_length;
    double constraint_violation;
    double defect_norm;

    ForwardPassResult(int nx, int nu, int N);
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



IterativeLQR::IterativeLQR(cs::Function fdyn,
                           int N):
    _nx(fdyn.size1_in(0)),
    _nu(fdyn.size1_in(1)),
    _N(N),
    _cost(N+1, IntermediateCost(_nx, _nu)),
    _constraint(N+1),
    _value(N+1, ValueFunction(_nx)),
    _dyn(N, Dynamics(_nx, _nu)),
    _bp_res(N, BackwardPassResult(_nx, _nu)),
    _constraint_to_go(std::make_unique<ConstraintToGo>(_nx, _nu)),
    _fp_res(std::make_unique<ForwardPassResult>(_nx, _nu, _N)),
    _tmp(_N)
{
    // set timer callback
    on_timer_toc = [this](const char * name, double usec)
    {
        _prof_info.timings[name].push_back(usec);
    };

    // set dynamics
    for(auto& d : _dyn)
    {
        d.setDynamics(fdyn);
    }

    // initialize trajectories
    _xtrj.setZero(_nx, _N+1);
    _utrj.setZero(_nu, _N);

    // a default cost so that it works out of the box
    set_default_cost();
}


void IterativeLQR::setIntermediateCost(const std::vector<casadi::Function> &inter_cost)
{
    if(inter_cost.size() != _N)
    {
        throw std::invalid_argument("wrong intermediate cost length");
    }

    for(int i = 0; i < _N; i++)
    {
        _cost[i].setCost(inter_cost[i]);
    }
}

void IterativeLQR::setIntermediateCost(int k, const casadi::Function &inter_cost)
{
    if(k > _N || k < 0)
    {
        throw std::invalid_argument("wrong intermediate cost node index");
    }

    _cost[k].setCost(inter_cost);
}

void IterativeLQR::setFinalCost(const casadi::Function &final_cost)
{
    _cost.back().setCost(final_cost);
}

void IterativeLQR::setIntermediateConstraint(int k, const casadi::Function &inter_constraint)
{
    if(k > _N || k < 0)
    {
        throw std::invalid_argument("wrong intermediate constraint node index");
    }

    _constraint[k].setConstraint(inter_constraint);
}

void IterativeLQR::setIntermediateConstraint(const std::vector<casadi::Function> &inter_constraint)
{
    if(inter_constraint.size() != _N)
    {
        throw std::invalid_argument("wrong intermediate constraint length");
    }

    for(int i = 0; i < _N; i++)
    {
        _constraint[i].setConstraint(inter_constraint[i]);
    }
}

void IterativeLQR::setFinalConstraint(const casadi::Function &final_constraint)
{
    _constraint.back().setConstraint(final_constraint);
}

void IterativeLQR::setInitialState(const Eigen::VectorXd &x0)
{
    if(x0.size() != _nx)
    {
        throw std::invalid_argument("wrong initial state length");
    }

    _xtrj.col(0) = x0;
}

void IterativeLQR::setStateInitialGuess(const Eigen::MatrixXd& x0)
{
    if(x0.rows() != _xtrj.rows())
    {
        throw std::invalid_argument("wrong initial guess rows");
    }

    if(x0.cols() != _xtrj.cols())
    {
        throw std::invalid_argument("wrong initial guess cols");
    }

    _xtrj = x0;
}

void IterativeLQR::setIterationCallback(const CallbackType &cb)
{
    _iter_cb = cb;
}

const Eigen::MatrixXd &IterativeLQR::getStateTrajectory() const
{
    return _xtrj;
}

const Eigen::MatrixXd &IterativeLQR::getInputTrajectory() const
{
    return _utrj;
}

const utils::ProfilingInfo& IterativeLQR::getProfilingInfo() const
{
    return _prof_info;
}

bool IterativeLQR::solve(int max_iter)
{
    // tbd implement convergence check

    for(int i = 0; i < max_iter; i++)
    {
        TIC(solve)

        linearize_quadratize();
        backward_pass();
        forward_pass(1.0);

        TOC(solve)

        report_result();
    }

    return true;
}

void IterativeLQR::linearize_quadratize()
{
    TIC(linearize_quadratize)

    for(int i = 0; i < _N; i++)
    {
        TIC(linearize_quadratize_inner)

        auto xi = state(i);
        auto ui = input(i);
        auto xnext = state(i+1);

        _dyn[i].linearize(xi, ui);
        _dyn[i].computeDefect(xi, ui, xnext);
        _constraint[i].linearize(xi, ui);
        _cost[i].quadratize(xi, ui);
    }

    // handle final cost and constraint
    // note: these are only function of the state!
    _cost.back().quadratize(state(_N), input(_N-1)); // note: input not used here!
    _constraint.back().linearize(state(_N), input(_N-1)); // note: input not used here!
}

void IterativeLQR::report_result()
{
    if(!_iter_cb)
    {
        return;
    }

    // call iteration callback
    _iter_cb(_xtrj,
             _utrj,
             _fp_res->step_length,
             _fp_res->cost,
             _fp_res->defect_norm,
             _fp_res->constraint_violation);
}

void IterativeLQR::backward_pass()
{
    TIC(backward_pass)

    // initialize backward recursion from final cost..
    _value.back().S = _cost.back().Q();
    _value.back().s = _cost.back().q();

    // ..and constraint
    _constraint_to_go->set(_constraint.back());

    // backward pass
    for(int i = _N-1; i >= 0; i--)
    {
        backward_pass_iter(i);
    }
}

void IterativeLQR::backward_pass_iter(int i)
{
    TIC(backward_pass_inner)

    // constraint handling
    auto [nz, cdyn, ccost] = handle_constraints(i);

    const bool has_constraints = nz != _nu;

    // note: after handling constraints, we're actually optimizing an
    // auxiliary input z, where the original input u = lc + Lc*x + Lz*z

    // some shorthands

    // value function
    const auto& value_next = _value[i+1];
    const auto& Snext = value_next.S;
    const auto& snext = value_next.s;

    // intermediate cost
    const auto r = ccost.r;
    const auto q = ccost.q;
    const auto Q = ccost.Q;
    const auto R = ccost.R;
    const auto P = ccost.P;

    // dynamics
    const auto A = cdyn.A;
    const auto B = cdyn.B;
    const auto d = cdyn.d;

    // workspace
    auto& tmp = _tmp[i];

    // mapping to original input u
    const auto& lc = tmp.lc;
    const auto& Lc = tmp.Lc;
    const auto& Lz = tmp.Lz;

    // components of next node's value function (as a function of
    // current state and control via the dynamics)
    // note: first compute state-only components, since after constraints
    // there might be no input dof to optimize at all!
    tmp.s_plus_S_d.noalias() = snext + Snext*d;
    tmp.S_A.noalias() = Snext*A;

    tmp.hx.noalias() = q + A.transpose()*tmp.s_plus_S_d;
    tmp.Hxx.noalias() = Q + A.transpose()*tmp.S_A;


    // handle case where nz = 0, i.e. no nullspace left after constraints
    if(nz == 0)
    {
        // save solution
        auto& res = _bp_res[i];
        auto& L = res.Lfb;
        auto& l = res.du_ff;
        L = Lc;
        l = lc;

        // save optimal value function
        auto& value = _value[i];
        auto& S = value.S;
        auto& s = value.s;
        S = tmp.Hxx;
        s = tmp.hx;

        return;
    }

    // remaining components of next node's value function (if nz > 0)
    tmp.hu.noalias() = r + B.transpose()*tmp.s_plus_S_d;
    tmp.Huu.noalias() = R + B.transpose()*Snext*B;
    tmp.Hux.noalias() = P + B.transpose()*tmp.S_A;

    // set huHux = [hu Hux]
    tmp.huHux.resize(nz, 1+_nx);
    tmp.huHux.col(0) = tmp.hu;
    tmp.huHux.rightCols(_nx) = tmp.Hux;

    // todo: second-order terms from dynamics

    // solve linear system to get ff and fb terms
    // after solveInPlace we will have huHux = [-l, -L]
    tmp.llt.compute(tmp.Huu);
    tmp.llt.solveInPlace(tmp.huHux);

    // todo: check solution for nan, unable to solve, etc

    // save solution
    auto& res = _bp_res[i];
    auto& L = res.Lfb;
    auto& l = res.du_ff;
    L = -tmp.huHux.rightCols(_nx);
    l = -tmp.huHux.col(0);

    // save optimal value function
    auto& value = _value[i];
    auto& S = value.S;
    auto& s = value.s;

    S.noalias() = tmp.Hxx - L.transpose()*tmp.Huu*L;
    s.noalias() = tmp.hx + tmp.Hux.transpose()*l + L.transpose()*(tmp.hu + tmp.Huu*l);

    // map to original input u
    if(has_constraints)
    {
        l = lc + Lz*l;
        L = Lc + Lz*L;
    }

}

IterativeLQR::HandleConstraintsRetType IterativeLQR::handle_constraints(int i)
{
    TIC(handle_constraints_inner)

    // some shorthands for..

    // ..intermediate cost
    const auto& cost = _cost[i];
    const auto r = cost.r();
    const auto q = cost.q();
    const auto& Q = cost.Q();
    const auto& R = cost.R();
    const auto& P = cost.P();

    // ..dynamics
    auto& dyn = _dyn[i];
    const auto& A = dyn.A();
    const auto& B = dyn.B();
    const auto& d = dyn.d;

    // ..workspace
    auto& tmp = _tmp[i];
    auto& svd = tmp.svd;
    auto& rotC = tmp.rotC;
    auto& roth = tmp.roth;
    auto& lc = tmp.lc;
    auto& Lc = tmp.Lc;
    auto& Lz = tmp.Lz;

    // no constraint to handle, do nothing
    if(_constraint_to_go->dim() == 0 &&
            !_constraint[i].is_valid())
    {
        ConstrainedDynamics cd = {A, B, d};
        ConstrainedCost cc = {Q, R, P, q, r};
        return std::make_tuple(_nu, cd, cc);
    }

    // back-propagate constraint to go from next step to current step
    _constraint_to_go->propagate_backwards(A, B, d);

    // add current step intermediate constraint
    _constraint_to_go->add(_constraint[i]);
    auto C = _constraint_to_go->C();
    auto D = _constraint_to_go->D();
    auto h = _constraint_to_go->h();

    // number of constraints
    int nc = _constraint_to_go->dim();

    // svd of input matrix
    const double sv_ratio_thr = 1e-3;
    svd.compute(D, Eigen::ComputeFullU|Eigen::ComputeFullV);
    const auto& U = svd.matrixU();
    const auto& V = svd.matrixV();
    const auto& sv = svd.singularValues();
    svd.setThreshold(sv[0]*sv_ratio_thr);
    int rank = svd.rank();
    int ns_dim = _nu - rank;

    // rotate constraints
    rotC.noalias() = U.transpose()*C;
    roth.noalias() = U.transpose()*h;

    // compute component of control input due to constraints,
    // i.e. uc = Lc*x + +Lz*z + lc, where:
    //  *) lc = -V[:, 0:r]*sigma^-1*rot_h
    //  *) Lz = V[:, r:]
    //  *) Lc = -V[:, 0:r]*sigma^-1*rot_C
    lc.noalias() = -V.leftCols(rank) * roth.head(rank).cwiseQuotient(sv.head(rank));
    Lc.noalias() = -V.leftCols(rank) * sv.head(rank).cwiseInverse().asDiagonal() * rotC.topRows(rank);
    Lz.noalias() = V.rightCols(ns_dim);

    // remove satisfied constraints from constraint to go
    _constraint_to_go->set(rotC.bottomRows(nc - rank),
                           roth.tail(nc - rank));

    // modified cost and dynamics due to uc = uc(x, z)
    // note: our new control input will be z!
    tmp.Ac.noalias() = A + B*Lc;
    tmp.Bc.noalias() = B*Lz;
    tmp.dc.noalias() = d + B*lc;

    tmp.qc.noalias() = q + Lc.transpose()*(r + R*lc) + P.transpose()*lc;
    tmp.rc.noalias() = Lz.transpose()*(r + R*lc);
    tmp.Qc.noalias() = Q + Lc.transpose()*R*Lc + Lc.transpose()*P + P.transpose()*Lc;
    tmp.Rc.noalias() = Lz.transpose()*R*Lz;
    tmp.Pc.noalias() = Lz.transpose()*(P + R*Lc);

    // return
    ConstrainedDynamics cd = {tmp.Ac, tmp.Bc, tmp.dc};
    ConstrainedCost cc = {tmp.Qc, tmp.Rc, tmp.Pc, tmp.qc, tmp.rc};
    return std::make_tuple(ns_dim, cd, cc);

}

bool IterativeLQR::forward_pass(double alpha)
{
    TIC(forward_pass)

    // reset cost
    _fp_res->cost = 0.0;
    _fp_res->defect_norm = 0.0;
    _fp_res->constraint_violation = 0.0;
    _fp_res->step_length = 0.0;

    // initialize forward pass with initial state
    _fp_res->xtrj.col(0) = _xtrj.col(0);

    // do forward pass
    for(int i = 0; i < _N; i++)
    {
        forward_pass_iter(i, alpha);
    }

    // compute final constraint violation
    if(_constraint[_N].is_valid())
    {
        // note: u not used
        // todo: enforce this!
        _constraint[_N].evaluate(_fp_res->xtrj.col(_N), _fp_res->utrj.col(_N-1));
        _fp_res->constraint_violation += _constraint[_N].h().cwiseAbs().sum();
    }

    // todo: add line search
    // for now, we always accept the step
    _xtrj = _fp_res->xtrj;
    _utrj = _fp_res->utrj;

    return true;
}

void IterativeLQR::forward_pass_iter(int i, double alpha)
{
    TIC(forward_pass_inner)

    // note!
    // this function will update the control at t = i, and
    // the state at t = i+1

    // some shorthands
    const auto xnext = state(i+1);
    const auto xi = state(i);
    const auto ui = input(i);
    const auto xi_upd = _fp_res->xtrj.col(i);
    auto& tmp = _tmp[i];
    tmp.dx = xi_upd - xi;

    // dynamics
    const auto& dyn = _dyn[i];
    const auto& A = dyn.A();
    const auto& B = dyn.B();
    const auto& d = dyn.d;

    // backward pass solution
    const auto& res = _bp_res[i];
    const auto& L = res.Lfb;
    auto l = alpha * res.du_ff;

    // update control
    auto ui_upd = ui + l + L*tmp.dx;
    _fp_res->utrj.col(i) = ui_upd;

    // update next state
    auto xnext_upd = xnext + (A + B*L)*tmp.dx + B*l + d;
    _fp_res->xtrj.col(i+1) = xnext_upd;

    // compute cost on current trajectory
    double cost = _cost[i].evaluate(_fp_res->xtrj.col(i), _fp_res->utrj.col(i));
    _fp_res->cost += cost;

    // compute defects on current trajectory
    _dyn[i].computeDefect(_fp_res->xtrj.col(i), _fp_res->utrj.col(i), _fp_res->xtrj.col(i+1));
    _fp_res->defect_norm += _dyn[i].d.cwiseAbs().sum();

    // compute constraint violation on current trajectory
    if(_constraint[i].is_valid())
    {
        _constraint[i].evaluate(_fp_res->xtrj.col(i), _fp_res->utrj.col(i));
        _fp_res->constraint_violation += _constraint[i].h().cwiseAbs().sum();
    }

    // compute step length
    _fp_res->step_length += l.cwiseAbs().sum();

}

void IterativeLQR::set_default_cost()
{
    auto x = cs::SX::sym("x", _nx);
    auto u = cs::SX::sym("u", _nu);
    auto l = cs::Function("dfl_cost", {x, u},
                          {0.5*cs::SX::sumsqr(u)},
                          {"x", "u"}, {"l"});
    auto lf = cs::Function("dfl_cost_final", {x, u}, {0.5*cs::SX::sumsqr(x)},
                           {"x", "u"}, {"l"});
    setIntermediateCost(std::vector<cs::Function>(_N, l));
    setFinalCost(lf);
}

VecConstRef IterativeLQR::state(int i) const
{
    return _xtrj.col(i);
}

VecConstRef IterativeLQR::input(int i) const
{
    return _utrj.col(i);
}

const Eigen::MatrixXd &IterativeLQR::Dynamics::A() const
{
    return df.getOutput(0);
}

const Eigen::MatrixXd &IterativeLQR::Dynamics::B() const
{
    return df.getOutput(1);
}

IterativeLQR::Dynamics::Dynamics(int nx, int)
{
    d.setZero(nx);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::Dynamics::integrate(VecConstRef x, VecConstRef u)
{
    f.setInput(0, x);
    f.setInput(1, u);
    f.call();
    return f.getOutput(0);
}

void IterativeLQR::Dynamics::linearize(VecConstRef x, VecConstRef u)
{
    TIC(linearize_dynamics_inner)
    df.setInput(0, x);
    df.setInput(1, u);
    df.call();
}

void IterativeLQR::Dynamics::computeDefect(VecConstRef x, VecConstRef u, VecConstRef xnext)
{
    TIC(compute_defect_inner)
    auto xint = integrate(x, u);
    d = xint - xnext;
}

void IterativeLQR::Dynamics::setDynamics(casadi::Function _f)
{
    f = _f;
    df = _f.factory("df", {"x", "u"}, {"jac:f:x", "jac:f:u"});
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCost::Q() const
{
    return ddl.getOutput(0);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::IntermediateCost::q() const
{
    return dl.getOutput(0).col(0);
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCost::R() const
{
    return ddl.getOutput(1);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::IntermediateCost::r() const
{
    return dl.getOutput(1).col(0);
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCost::P() const
{
    return ddl.getOutput(2);
}

IterativeLQR::IntermediateCost::IntermediateCost(int, int)
{
}

void IterativeLQR::IntermediateCost::setCost(const casadi::Function &cost)
{
    l = cost;

    // note: use grad to obtain a column vector!
    dl = l.function().factory("dl", {"x", "u"}, {"grad:l:x", "grad:l:u"});
    ddl = dl.function().factory("ddl", {"x", "u"}, {"jac:grad_l_x:x", "jac:grad_l_u:u", "jac:grad_l_u:x"});

    // tbd: do something with this
    bool is_quadratic = ddl.function().jacobian().nnz_out() == 0;
    static_cast<void>(is_quadratic);
}

double IterativeLQR::IntermediateCost::evaluate(VecConstRef x, VecConstRef u)
{
    // compute cost value
    l.setInput(0, x);
    l.setInput(1, u);
    l.call();

    return l.getOutput(0).value();
}

void IterativeLQR::IntermediateCost::quadratize(VecConstRef x, VecConstRef u)
{
    TIC(quadratize_inner)

    // compute cost gradient
    dl.setInput(0, x);
    dl.setInput(1, u);
    dl.call();

    // compute cost hessian
    ddl.setInput(0, x);
    ddl.setInput(1, u);
    ddl.call();

}

IterativeLQR::ValueFunction::ValueFunction(int nx)
{
    S.setZero(nx, nx);
    s.setZero(nx);
}

IterativeLQR::BackwardPassResult::BackwardPassResult(int nx, int nu)
{
    Lfb.setZero(nu, nx);
    du_ff.setZero(nu);
}

IterativeLQR::ForwardPassResult::ForwardPassResult(int nx, int nu, int N)
{
    xtrj.setZero(nx, N+1);
    utrj.setZero(nu, N);
}

IterativeLQR::ConstraintToGo::ConstraintToGo(int nx, int nu):
    _dim(0)
{
    const int c_max = nx*10;  // todo: better estimate
    _C.setZero(c_max, nx);
    _h.setZero(c_max);
    _D.setZero(c_max, nu);
}

void IterativeLQR::ConstraintToGo::set(MatConstRef C, VecConstRef h)
{
    _dim = h.size();
    _C.topRows(_dim) = C;
    _h.head(_dim) = h;
    _D.array().topRows(_dim) = 0;  // note! we assume no input dependence here!
}

void IterativeLQR::ConstraintToGo::set(const IterativeLQR::Constraint &constr)
{
    if(!constr.is_valid())
    {
        return;
    }

    _dim = constr.h().size();
    _C.topRows(_dim) = constr.C();
    _h.head(_dim) = constr.h();
    _D.array().topRows(_dim) = 0;  // note! we assume no input dependence here!
}

void IterativeLQR::ConstraintToGo::propagate_backwards(MatConstRef A,
                                                       MatConstRef B,
                                                       VecConstRef d)
{
    // tbd: avoid allocations

    _D.topRows(_dim) = C()*B;
    _h.head(_dim) = h() + C()*d;  // note: check sign!
    _C.topRows(_dim) = C()*A;  // note: C must be modified last!
}

void IterativeLQR::ConstraintToGo::add(const Constraint &constr)
{
    if(!constr.is_valid())
    {
        return;
    }

    const int constr_size = constr.h().size();
    _C.middleRows(_dim, constr_size) = constr.C();
    _D.middleRows(_dim, constr_size) = constr.D();
    _h.segment(_dim, constr_size) = constr.h();
    _dim += constr_size;
}

void IterativeLQR::ConstraintToGo::clear()
{
    _dim = 0;
}

int IterativeLQR::ConstraintToGo::dim() const
{
    return _dim;
}

Eigen::Ref<const Eigen::MatrixXd> IterativeLQR::ConstraintToGo::C() const
{
    return _C.topRows(_dim);
}

MatConstRef IterativeLQR::ConstraintToGo::D() const
{
    return _D.topRows(_dim);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::ConstraintToGo::h() const
{
    return _h.head(_dim);
}

const Eigen::MatrixXd &IterativeLQR::Constraint::C() const
{
    return df.getOutput(0);
}

const Eigen::MatrixXd &IterativeLQR::Constraint::D() const
{
    return df.getOutput(1);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::Constraint::h() const
{
    return f.getOutput(0).col(0);
}

bool IterativeLQR::Constraint::is_valid() const
{
    return f.is_valid();
}

IterativeLQR::Constraint::Constraint()
{
}

void IterativeLQR::Constraint::linearize(VecConstRef x, VecConstRef u)
{
    if(!is_valid())
    {
        return;
    }

    TIC(linearize_constraint_inner)

    // compute constraint value
    f.setInput(0, x);
    f.setInput(1, u);
    f.call();

    // compute constraint jacobian
    df.setInput(0, x);
    df.setInput(1, u);
    df.call();
}

void IterativeLQR::Constraint::evaluate(VecConstRef x, VecConstRef u)
{
    if(!is_valid())
    {
        return;
    }

    TIC(evaluate_constraint_inner)

    // compute constraint value
    f.setInput(0, x);
    f.setInput(1, u);
    f.call();
}

void IterativeLQR::Constraint::setConstraint(casadi::Function h)
{
    f = h;
    df = h.factory("dh", {"x", "u"}, {"jac:h:x", "jac:h:u"});
}

IterativeLQR::~IterativeLQR() = default;
