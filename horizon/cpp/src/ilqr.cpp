#include "ilqr_impl.h"

utils::Timer::TocCallback on_timer_toc;

IterativeLQR::IterativeLQR(cs::Function fdyn,
                           int N):
    _nx(fdyn.size1_in(0)),
    _nu(fdyn.size1_in(1)),
    _N(N),
    _step_length(1.0),
    _cost(N+1, IntermediateCost(_nx, _nu)),
    _constraint(N+1),
    _value(N+1, ValueFunction(_nx)),
    _dyn(N, Dynamics(_nx, _nu)),
    _bp_res(N, BackwardPassResult(_nx, _nu)),
    _constraint_to_go(std::make_unique<ConstraintToGo>(_nx, _nu)),
    _fp_res(std::make_unique<ForwardPassResult>(_nx, _nu, _N)),
    _lam_g(_N+1),
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
    _lam_x.setZero(_nx, _N);


    // a default cost so that it works out of the box
    //  *) default intermediate cost -> l(x, u) = eps*|u|^2
    //  *) default final cost        -> lf(x)   = eps*|xf|^2
    set_default_cost();
}

void IterativeLQR::setStepLength(double alpha)
{
    _step_length = alpha;
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

void IterativeLQR::setInputInitialGuess(const Eigen::MatrixXd &u0)
{
    if(u0.rows() != _utrj.rows())
    {
        throw std::invalid_argument("wrong initial guess rows");
    }

    if(u0.cols() != _utrj.cols())
    {
        throw std::invalid_argument("wrong initial guess cols");
    }

    _utrj = u0;
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
    // set cost value and constraint violation *before* the forward pass
    _fp_res->cost = compute_cost(_xtrj, _utrj);
    _fp_res->constraint_violation = compute_constr(_xtrj, _utrj);
    _fp_res->defect_norm = compute_defect(_xtrj, _utrj);

    // solve
    for(int i = 0; i < max_iter; i++)
    {
        TIC(solve);

        _fp_res->iter = i;

        linearize_quadratize();
        backward_pass();
        line_search(i);

        if(should_stop())
        {
            return true;
        }
    }

    return false;
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
        _dyn[i].computeDefect(xi, ui, xnext, _dyn[i].d);
        _constraint[i].linearize(xi, ui);
        _cost[i].quadratize(xi, ui);
    }

    // handle final cost and constraint
    // note: these are only function of the state!
    _cost.back().quadratize(state(_N), input(_N-1)); // note: input not used here!
    _constraint.back().linearize(state(_N), input(_N-1)); // note: input not used here!
}

void IterativeLQR::report_result(const IterativeLQR::ForwardPassResult& fpres)
{
    if(!_iter_cb)
    {
        return;
    }

    // call iteration callback
    _iter_cb(fpres);
}

void IterativeLQR::set_default_cost()
{
    const double dfl_cost_weight = 1e-6;
    auto x = cs::SX::sym("x", _nx);
    auto u = cs::SX::sym("u", _nu);
    auto l = cs::Function("dfl_cost", {x, u},
                          {dfl_cost_weight*cs::SX::sumsqr(u)},
                          {"x", "u"}, {"l"});
    auto lf = cs::Function("dfl_cost_final", {x, u}, {dfl_cost_weight*cs::SX::sumsqr(x)},
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

void IterativeLQR::Dynamics::computeDefect(VecConstRef x,
                                           VecConstRef u,
                                           VecConstRef xnext,
                                           Eigen::VectorXd& _d)
{
    TIC(compute_defect_inner)
    auto xint = integrate(x, u);
    _d = xint - xnext;
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
    Lu.setZero(nu, nx);
    lu.setZero(nu);
}

IterativeLQR::ForwardPassResult::ForwardPassResult(int nx, int nu, int N):
    alpha(0),
    accepted(false)
{
    xtrj.setZero(nx, N+1);
    utrj.setZero(nu, N);
    merit = 0.0;
    step_length = 0.0;
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
        clear();
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
