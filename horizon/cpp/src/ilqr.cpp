#include "ilqr_impl.h"
#include <cxxabi.h>

utils::Timer::TocCallback on_timer_toc;

template<class V>
std::type_info const& var_type(V const& v){
  return std::visit( [](auto&&x)->decltype(auto){ return typeid(x); }, v );
}

template <typename T>
T value_or(const IterativeLQR::OptionDict& opt, std::string key, T dfl)
{
    auto it = opt.find(key);

    if(it == opt.end())
    {
        return dfl;
    }

    try
    {
        return std::get<T>(it->second);
    }
    catch(std::bad_variant_access& e)
    {
        throw std::runtime_error(
                    std::string("bad type '") +
                    abi::__cxa_demangle(var_type(it->second).name(), 0, 0, 0) +
                    "' for parameter '" + key + "': expected " +
                    abi::__cxa_demangle(typeid(T).name(), 0, 0, 0)
                    );
    }
}

IterativeLQR::IterativeLQR(cs::Function fdyn,
                           int N,
                           OptionDict opt):
    _nx(fdyn.size1_in(0)),
    _nu(fdyn.size1_in(1)),
    _N(N),
    _step_length(1.0),
    _hxx_reg(0.0),
    _hxx_reg_growth_factor(1e3),
    _line_search_accept_ratio(1e-4),
    _alpha_min(1e-3),
    _svd_threshold(1e-6),
    _cost(N+1, IntermediateCost(_nx, _nu)),
    _constraint(N+1, Constraint(_nx, _nu)),
    _value(N+1, ValueFunction(_nx)),
    _dyn(N, Dynamics(_nx, _nu)),
    _bp_res(N, BackwardPassResult(_nx, _nu)),
    _constraint_to_go(std::make_unique<ConstraintToGo>(_nx, _nu)),
    _fp_res(std::make_unique<ForwardPassResult>(_nx, _nu, _N)),
    _lam_g(_N+1),
    _tmp(_N)
{
    // set options
    _step_length = value_or(opt, "ilqr.step_length", 1.0);
    _hxx_reg = value_or(opt, "ilqr.hxx_reg", 0.0);
    _hxx_reg_growth_factor = value_or(opt, "ilqr.hxx_reg_growth_factor", 1e3);
    _line_search_accept_ratio = value_or(opt, "ilqr.line_search_accept_ratio", 1e-4);
    _alpha_min = value_or(opt, "ilqr.alpha_min", 1e-3);
    _svd_threshold = value_or(opt, "ilqr.svd_threshold", 1e-6);
    _closed_loop_forward_pass = value_or(opt, "ilqr.closed_loop_forward_pass", 1);

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
        _cost[i].addCost(inter_cost[i]);
    }
}

void IterativeLQR::setCost(std::vector<int> indices, const casadi::Function& inter_cost)
{
    IntermediateCostEntity c;
    auto grad = c.Gradient(inter_cost);
    c.setCost(inter_cost,
              grad,
              c.Hessian(grad));

    std::cout << "adding cost '" << inter_cost << "' at k = ";

    for(int k : indices)
    {
        if(k > _N || k < 0)
        {
            throw std::invalid_argument("wrong intermediate cost node index");
        }

        std::cout << k << " ";

        _cost[k].addCost(c);
    }

    std::cout << "\n";
}

void IterativeLQR::setFinalCost(const casadi::Function &final_cost)
{
    _cost.back().addCost(final_cost);
}

void IterativeLQR::setConstraint(std::vector<int> indices,
                                 const casadi::Function &inter_constraint,
                                 std::vector<Eigen::VectorXd> target_values)
{
    ConstraintEntity c;
    c.setConstraint(inter_constraint,
                    c.Jacobian(inter_constraint));

    std::cout << "adding constraint '" << inter_constraint << "' at k = ";

    for(int i = 0; i < indices.size(); i++)
    {
        const int k = indices[i];

        if(k > _N || k < 0)
        {
            throw std::invalid_argument("wrong intermediate constraint node index");
        }

        if(target_values.size() > 0)
        {
            c.setTargetValue(target_values[i]);
        }

        std::cout << k << " ";

        _constraint[k].addConstraint(c);
    }

    std::cout << "\n";
}

void IterativeLQR::setIntermediateConstraint(const std::vector<casadi::Function> &inter_constraint)
{
    if(inter_constraint.size() != _N)
    {
        throw std::invalid_argument("wrong intermediate constraint length");
    }

    for(int i = 0; i < _N; i++)
    {
        _constraint[i].addConstraint(inter_constraint[i]);
    }
}

void IterativeLQR::setFinalConstraint(const casadi::Function &final_constraint)
{
    _constraint.back().addConstraint(final_constraint);
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
    const double dfl_cost_weight = 1e-160;
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

MatConstRef IterativeLQR::gain(int i) const
{
    return _bp_res[i].Lu;
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

const Eigen::MatrixXd& IterativeLQR::IntermediateCostEntity::Q() const
{
    return ddl.getOutput(0);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::IntermediateCostEntity::q() const
{
    return dl.getOutput(0).col(0);
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCostEntity::R() const
{
    return ddl.getOutput(1);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::IntermediateCostEntity::r() const
{
    return dl.getOutput(1).col(0);
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCostEntity::P() const
{
    return ddl.getOutput(2);
}

void IterativeLQR::IntermediateCostEntity::setCost(const casadi::Function &cost)
{
    l = cost;

    // note: use grad to obtain a column vector!
    dl = Gradient(cost);
    ddl = Hessian(dl.function());

    // tbd: do something with this
    // bool is_quadratic = ddl.function().jacobian().nnz_out() == 0;
    // static_cast<void>(is_quadratic);
}

void IterativeLQR::IntermediateCostEntity::setCost(const casadi::Function &f,
                                                   const casadi::Function &df,
                                                   const casadi::Function &ddf)
{
    l = f;
    dl = df;
    ddl = ddf;
}

double IterativeLQR::IntermediateCostEntity::evaluate(VecConstRef x, VecConstRef u)
{
    // compute cost value
    l.setInput(0, x);
    l.setInput(1, u);
    l.call();

    return l.getOutput(0).value();
}

void IterativeLQR::IntermediateCostEntity::quadratize(VecConstRef x, VecConstRef u)
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

casadi::Function IterativeLQR::IntermediateCostEntity::Gradient(const casadi::Function& f)
{
    return f.factory("dl", {"x", "u"}, {"grad:l:x", "grad:l:u"});
}

casadi::Function IterativeLQR::IntermediateCostEntity::Hessian(const casadi::Function& df)
{
    return df.factory("ddl", {"x", "u"}, {"jac:grad_l_x:x", "jac:grad_l_u:u", "jac:grad_l_u:x"});
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCost::Q() const
{
    return _Q;
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCost::R() const
{
    return _R;
}

const Eigen::MatrixXd& IterativeLQR::IntermediateCost::P() const
{
    return _P;
}

VecConstRef IterativeLQR::IntermediateCost::q() const
{
    return _q;
}

VecConstRef IterativeLQR::IntermediateCost::r() const
{
    return _r;
}

IterativeLQR::IntermediateCost::IntermediateCost(int nx, int nu)
{
    _Q.setZero(nx, nx);
    _R.setZero(nu, nu);
    _P.setZero(nu, nx);
    _q.setZero(nx);
    _r.setZero(nu);
}

void IterativeLQR::IntermediateCost::addCost(const casadi::Function &cost)
{
    items.emplace_back();
    items.back().setCost(cost);
}

void IterativeLQR::IntermediateCost::addCost(const IntermediateCostEntity &cost)
{
    items.emplace_back(cost);
}

double IterativeLQR::IntermediateCost::evaluate(VecConstRef x, VecConstRef u)
{
    double cost = 0.0;

    for(auto& it : items)
    {
        cost += it.evaluate(x, u);
    }

    return cost;
}

void IterativeLQR::IntermediateCost::quadratize(VecConstRef x, VecConstRef u)
{
    TIC(quadratize_inner)

    const int nx = _Q.rows();
    const int nu = _R.rows();

    _Q.setZero(nx, nx);
    _R.setZero(nu, nu);
    _P.setZero(nu, nx);
    _q.setZero(nx);
    _r.setZero(nu);

    // todo: can optimize the first sum

    for(auto& it : items)
    {
        it.quadratize(x, u);
        _Q += it.Q();
        _R += it.R();
        _P += it.P();
        _q += it.q();
        _r += it.r();
    }
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
    hxx_reg(0),
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

    if(_dim + constr_size >= _h.size())
    {
        throw std::runtime_error("maximum constraint-to-go dimension "
            "exceeded: try reducing the svd_threshold parameter");
    }

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

const Eigen::MatrixXd &IterativeLQR::ConstraintEntity::C() const
{
    return df.getOutput(0);
}

const Eigen::MatrixXd &IterativeLQR::ConstraintEntity::D() const
{
    return df.getOutput(1);
}

Eigen::Ref<const Eigen::VectorXd> IterativeLQR::ConstraintEntity::h() const
{
    return _hvalue;
}

bool IterativeLQR::ConstraintEntity::is_valid() const
{
    return f.is_valid();
}

IterativeLQR::ConstraintEntity::ConstraintEntity()
{
}

void IterativeLQR::ConstraintEntity::linearize(VecConstRef x, VecConstRef u)
{
    if(!is_valid())
    {
        return;
    }

    // compute constraint value
    f.setInput(0, x);
    f.setInput(1, u);
    f.call();

    // compute constraint jacobian
    df.setInput(0, x);
    df.setInput(1, u);
    df.call();
}

void IterativeLQR::ConstraintEntity::evaluate(VecConstRef x, VecConstRef u)
{
    if(!is_valid())
    {
        return;
    }

    // compute constraint value
    f.setInput(0, x);
    f.setInput(1, u);
    f.call();

    // remove target value and save it
    _hvalue = f.getOutput(0).col(0) - _hdes;
}

void IterativeLQR::ConstraintEntity::setConstraint(casadi::Function h)
{
    f = h;
    df = Jacobian(h);
    _hdes.setZero(f.function().size1_out(0));
}

void IterativeLQR::ConstraintEntity::setConstraint(casadi::Function h, casadi::Function dh)
{
    f = h;
    df = dh;
    _hdes.setZero(f.function().size1_out(0));
}

void IterativeLQR::ConstraintEntity::setTargetValue(const Eigen::VectorXd &hdes)
{
    if(hdes.size() != _hdes.size())
    {
        throw std::invalid_argument("target value size mismatch");
    }

    _hdes = hdes;
}

casadi::Function IterativeLQR::ConstraintEntity::Jacobian(const casadi::Function& h)
{
    return h.factory("dh", {"x", "u"}, {"jac:h:x", "jac:h:u"});
}

const Eigen::MatrixXd &IterativeLQR::Constraint::C() const
{
    return _C;
}

const Eigen::MatrixXd &IterativeLQR::Constraint::D() const
{
    return _D;
}

VecConstRef IterativeLQR::Constraint::h() const
{
    return _h;
}

int IterativeLQR::Constraint::size() const
{
    return _C.rows();
}

bool IterativeLQR::Constraint::is_valid() const
{
    return !items.empty();
}

IterativeLQR::Constraint::Constraint(int nx, int nu)
{
    _C.setZero(0, nx);
    _D.setZero(0, nu);
}

void IterativeLQR::Constraint::linearize(VecConstRef x, VecConstRef u)
{
    TIC(linearize_constraint_inner)

    int i = 0;
    for(auto& it : items)
    {
        int nc = it.C().rows();
        it.linearize(x, u);
        _C.middleRows(i, nc) = it.C();
        _D.middleRows(i, nc) = it.D();
        i += nc;
    }
}

void IterativeLQR::Constraint::evaluate(VecConstRef x, VecConstRef u)
{
    TIC(evaluate_constraint_inner)

    int i = 0;
    for(auto& it : items)
    {
        int nc = it.C().rows();
        it.evaluate(x, u);
        _h.segment(i, nc) = it.h();
        i += nc;
    }
}

void IterativeLQR::Constraint::addConstraint(casadi::Function h)
{
    items.emplace_back();
    items.back().setConstraint(h);

    int total_size = size();
    total_size += h.size1_out(0);

    _C.setZero(total_size, _C.cols());
    _D.setZero(total_size, _D.cols());
    _h.setZero(total_size);
}

void IterativeLQR::Constraint::addConstraint(const ConstraintEntity &h)
{
    items.emplace_back(h);

    int total_size = size();
    total_size += h.f.function().size1_out(0);

    _C.setZero(total_size, _C.cols());
    _D.setZero(total_size, _D.cols());
    _h.setZero(total_size);
}

IterativeLQR::~IterativeLQR() = default;
