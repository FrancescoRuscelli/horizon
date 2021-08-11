#include "ilqr_impl.h"

bool IterativeLQR::forward_pass(double alpha)
{
    TIC(forward_pass)

    // reset cost
    _fp_res->alpha = alpha;
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

    // add final cost
    // note: u not used
    // todo: enforce this!
    double cost = _cost[_N].evaluate(_fp_res->xtrj.col(_N), _fp_res->utrj.col(_N-1));
    _fp_res->cost += cost;

    // add final constraint violation
    if(_constraint[_N].is_valid())
    {
        // note: u not used
        // todo: enforce this!
        _constraint[_N].evaluate(_fp_res->xtrj.col(_N), _fp_res->utrj.col(_N-1));
        _fp_res->constraint_violation += _constraint[_N].h().cwiseAbs().sum();
    }

    // tbd: better merit including constraints and defect
    _fp_res->merit = _fp_res->cost;

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
    const auto& L = res.Lu;
    auto l = alpha * res.lu;

    // update control
    auto ui_upd = ui + l + L*tmp.dx;
    _fp_res->utrj.col(i) = ui_upd;

    // update next state
    auto xnext_upd = xnext + (A + B*L)*tmp.dx + B*l + alpha*d;
    _fp_res->xtrj.col(i+1) = xnext_upd;

    // compute cost on current trajectory
    double cost = _cost[i].evaluate(_fp_res->xtrj.col(i), _fp_res->utrj.col(i));
    _fp_res->cost += cost;

    // compute defects on current trajectory
    _dyn[i].computeDefect(_fp_res->xtrj.col(i),
                          _fp_res->utrj.col(i),
                          _fp_res->xtrj.col(i+1),
                          tmp.defect);
    _fp_res->defect_norm += tmp.defect.cwiseAbs().sum();

    // compute constraint violation on current trajectory
    if(_constraint[i].is_valid())
    {
        _constraint[i].evaluate(_fp_res->xtrj.col(i), _fp_res->utrj.col(i));
        _fp_res->constraint_violation += _constraint[i].h().cwiseAbs().sum();
    }

    // compute step length
    _fp_res->step_length += l.cwiseAbs().sum();

}

void IterativeLQR::line_search()
{
    TIC(line_search);

    const double step_reduction_factor = 0.5 ;
    const double alpha_min = 0.001;
    double alpha = 1.0;

    ForwardPassResult best_res(_nx, _nu, _N);
    best_res.merit = std::numeric_limits<double>::max();

    while(alpha >= alpha_min)
    {
        forward_pass(alpha);

        if(_fp_res->merit < best_res.merit)
        {
            best_res = *_fp_res;
        }

        report_result(*_fp_res);
        alpha *= step_reduction_factor;
    }

    best_res.accepted = true;
    report_result(best_res);

    _xtrj = best_res.xtrj;
    _utrj = best_res.utrj;
}
