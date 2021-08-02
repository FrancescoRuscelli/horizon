#include "ilqr.h"

using namespace horizon;
using namespace casadi_utils;

namespace cs = casadi;

cs::DM to_cs(const Eigen::MatrixXd& eig)
{
    cs::DM ret(cs::Sparsity::dense(eig.rows(), eig.cols()));
    std::copy(eig.data(), eig.data() + eig.size(), ret.ptr());
    return ret;
}

Eigen::MatrixXd to_eig(const cs::DM& cas)
{
    auto cas_dense = cs::DM::densify(cas, 0);
    return Eigen::MatrixXd::Map(cas_dense.ptr(), cas_dense.size1(), cas_dense.size2());
}

IterativeLQR::IterativeLQR(cs::Function fdyn,
                           int N):
    _nx(fdyn.size1_in(0)),
    _nu(fdyn.size1_in(1)),
    _N(N),
    _f(fdyn),
    _cost(N+1, IntermediateCost(_nx, _nu)),
    _constraint(N+1),
    _value(N+1, ValueFunction(_nx)),
    _dyn(N, Dynamics(_nx, _nu)),
    _bp_res(N, BackwardPassResult(_nx, _nu)),
    _constraint_to_go(_nx),
    _fp_res(_nx, _nu, _N)
{
    // set dynamics
    for(auto& d : _dyn)
    {
        d.setDynamics(_f);
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

void IterativeLQR::setFinalCost(const casadi::Function &final_cost)
{
    _cost.back().setCost(final_cost);
}

void IterativeLQR::setInitialState(const Eigen::VectorXd &x0)
{
    _xtrj.col(0) = x0;
}

const Eigen::MatrixXd &IterativeLQR::getStateTrajectory() const
{
    return _xtrj;
}

const Eigen::MatrixXd &IterativeLQR::getInputTrajectory() const
{
    return _utrj;
}

void IterativeLQR::linearize_quadratize()
{
    for(int i = 0; i < _N; i++)
    {
        _dyn[i].linearize(_xtrj.col(i), _utrj.col(i));
    }

    for(int i = 0; i < _N; i++)
    {
        _cost[i].quadratize(_xtrj.col(i), _utrj.col(i));
    }

    _cost.back().quadratize(_xtrj.col(_N), _utrj.col(_N-1)); // note: input not used here!
}

void IterativeLQR::backward_pass()
{
    // initialize backward recursion from final cost..
    _value.back().S = _cost.back().Q();
    _value.back().s = _cost.back().q();

    // ..and constraint
    _constraint_to_go.clear();
    _constraint_to_go.add(_constraint.back().C(),
                          _constraint.back().h());

    // backward pass
    for(int i = _N-1; i >= 0; i--)
    {
        backward_pass_iter(i);
    }
}

void IterativeLQR::backward_pass_iter(int i)
{
    // some shorthands

    // value function
    const auto& value_next = _value[i+1];
    const auto& Snext = value_next.S;
    const auto& snext = value_next.s;

    // intermediate cost
    const auto& cost = _cost[i];
    const auto r = cost.r();
    const auto q = cost.q();
    const auto Q = cost.Q();
    const auto R = cost.R();
    const auto P = cost.P();

    // dynamics
    auto& dyn = _dyn[i];
    const auto A = dyn.A();
    const auto B = dyn.B();
    auto& d = dyn.d;


    // defect from integrated current state -> d = F(x_i, u_i) - xnext
    compute_defect(i, d);

    // components of next node's value function (as a function of
    // current state and control via the dynamics)
    tmp.s_plus_S_d.noalias() = snext + Snext*d;
    tmp.S_A.noalias() = Snext*A;

    tmp.hx.noalias() = q + A.transpose()*tmp.s_plus_S_d;
    tmp.hu.noalias() = r + B.transpose()*tmp.s_plus_S_d;
    tmp.Huu.noalias() = R + B.transpose()*Snext*B;
    tmp.Hxx.noalias() = Q + A.transpose()*tmp.S_A;
    tmp.Hux.noalias() = P + B.transpose()*tmp.S_A;

    // set huHux = [hu Hux]
    tmp.huHux.resize(_nu, 1+_nx);
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

}

bool IterativeLQR::forward_pass(double alpha)
{
    // start from current trajectory
    _fp_res.xtrj = _xtrj;
    _fp_res.utrj = _utrj;

    for(int i = 0; i < _N; i++)
    {
        forward_pass_iter(i, alpha);
    }

    // todo: add line search
    // for now, we always accept the step
    _xtrj = _fp_res.xtrj;
    _utrj = _fp_res.utrj;

    return true;
}

void IterativeLQR::forward_pass_iter(int i, double alpha)
{
    // note!
    // this function will update the control at t = i, and
    // the state at t = i+1

    // some shorthands
    const auto xnext = state(i+1);
    const auto xi = state(i);
    const auto ui = input(i);
    auto xi_upd = _fp_res.xtrj.col(i);
    tmp.dx = xi_upd - xi;

    // dynamics
    const auto& dyn = _dyn[i];
    const auto A = dyn.A();
    const auto B = dyn.B();
    const auto& d = dyn.d;

    // backward pass solution
    const auto& res = _bp_res[i];
    const auto& L = res.Lfb;
    auto l = alpha * res.du_ff;

    // update control
    auto ui_upd = ui + l + L*tmp.dx;
    _fp_res.utrj.col(i) = ui_upd;

    // update next state
    auto xnext_upd = xnext + (A + B*L)*tmp.dx + B*l + d;
    _fp_res.xtrj.col(i+1) = xnext_upd;
}

void IterativeLQR::compute_defect(int i, Eigen::VectorXd &d)
{
    auto xint = _dyn[i].integrate(state(i), input(i));
    d = xint - state(i+1);
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

Eigen::Ref<Eigen::VectorXd> IterativeLQR::state(int i)
{
    return _xtrj.col(i);
}

Eigen::Ref<Eigen::VectorXd> IterativeLQR::input(int i)
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

IterativeLQR::Dynamics::Dynamics(int nx, int nu)
{
    d.setZero(nx);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::Dynamics::integrate(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
    f.setInput(0, x);
    f.setInput(1, u);
    f.call();
    return f.getOutput(0);
}

void IterativeLQR::Dynamics::linearize(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
    df.setInput(0, x);
    df.setInput(1, u);
    df.call();
}

void IterativeLQR::Dynamics::setDynamics(casadi::Function _f)
{
    f = _f;
    df = _f.factory("df", {"x", "u"}, {"jac:f:x", "jac:f:u"});
}

Eigen::Ref<const Eigen::MatrixXd> IterativeLQR::IntermediateCost::Q() const
{
    return ddl.getOutput(0);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::IntermediateCost::q() const
{
    // note: gradient is a row vector for casadi
    return dl.getOutput(0).row(0);
}

Eigen::Ref<const Eigen::MatrixXd> IterativeLQR::IntermediateCost::R() const
{
    return ddl.getOutput(1);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::IntermediateCost::r() const
{
    // note: gradient is a row vector for casadi
    return dl.getOutput(1).row(0);
}

Eigen::Ref<const Eigen::MatrixXd> IterativeLQR::IntermediateCost::P() const
{
    return ddl.getOutput(2);
}

IterativeLQR::IntermediateCost::IntermediateCost(int nx, int nu)
{
}

void IterativeLQR::IntermediateCost::setCost(const casadi::Function &cost)
{
    l = cost;
    dl = l.function().factory("dl", {"x", "u"}, {"jac:l:x", "jac:l:u"});
    ddl = dl.function().factory("ddl", {"x", "u"}, {"jac:jac_l_x:x", "jac:jac_l_u:u", "jac:jac_l_u:x"});

    // tbd: do something with this
    bool is_quadratic = ddl.function().jacobian().nnz_out() == 0;
}

void IterativeLQR::IntermediateCost::quadratize(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
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
    xtrj.setZero(nx, N);
    utrj.setZero(nu, N);
}

IterativeLQR::ConstraintToGo::ConstraintToGo(int nx):
    dim(0)
{
    const int c_max = nx*10;  // todo: better estimate
    C.setZero(c_max, nx);
    h.setZero(c_max);
}

IterativeLQR::Constraint::Constraint()
{
    valid = false;
}

void IterativeLQR::Constraint::linearize(const Eigen::VectorXd& x, const Eigen::VectorXd& u)
{
    // compute constraint value
    f.setInput(0, x);
    f.setInput(1, u);
    f.call();

    // compute constraint jacobian
    df.setInput(0, x);
    df.setInput(1, u);
    df.call();
}

void IterativeLQR::Constraint::setConstraint(casadi::Function h)
{
    f = h;
    df = h.factory("dh", {"x", "u"}, {"jac:h:x", "jac:h:u"});
}
