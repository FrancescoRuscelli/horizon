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
    _cost(N+1, Cost(_nx, _nu)),
    _value(N+1, Cost(_nx, _nu)),
    _dyn(N, Dynamics(_nx, _nu)),
    _bp_res(N, BackwardPassResult(_nx, _nu)),
    _fp_res(_nx, _nu, _N)
{
    // compute expression for derivative of dynamics
    _df = _f.factory("df", {"x", "u"}, {"jac:f:x", "jac:f:u"});

    // set it to dynamics
    for(auto& d : _dyn)
    {
        d.f = _f;
        d.df = _df;
    }


    // initialize trajectories
    _xtrj.setZero(_nx, _N+1);
    _utrj.setZero(_nu, _N);

    // a default cost
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

    _cost.back().quadratize(_xtrj.col(_N), Eigen::VectorXd());
}

void IterativeLQR::backward_pass()
{
    // initialize backward recursion from final cost
    _value.back() = _cost.back();

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
    const auto& Snext = value_next.Q;
    const auto& snext = value_next.q;

    // intermediate cost
    const auto& cost = _cost[i];
    const auto& r = cost.r;
    const auto& q = cost.q;
    const auto& Q = cost.Q;
    const auto& R = cost.R;
    const auto& P = cost.P;

    // dynamics
    auto& dyn = _dyn[i];
    const auto& A = dyn.A;
    const auto& B = dyn.B;
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
    auto& S = value.Q;
    auto& s = value.q;

    S.noalias() = tmp.Hxx - L.transpose()*tmp.Huu*L;
    s.noalias() = tmp.hx + tmp.Hux.transpose()*l + L.transpose()*(tmp.hu + tmp.Huu*l);

}

bool IterativeLQR::forward_pass(double alpha)
{
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
    const auto& A = dyn.A;
    const auto& B = dyn.B;
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
    auto xint = _f(std::vector<cs::DM>{to_cs(state(i)), to_cs(input(i))})[0];
    d = to_eig(xint) - state(i+1);
}

Eigen::Ref<Eigen::VectorXd> IterativeLQR::state(int i)
{
    return _xtrj.col(i);
}

Eigen::Ref<Eigen::VectorXd> IterativeLQR::input(int i)
{
    return _utrj.col(i);
}

IterativeLQR::Dynamics::Dynamics(int nx, int nu)
{
    A.setZero(nx, nx);
    B.setZero(nx, nu);
    d.setZero(nx);
}

void IterativeLQR::Dynamics::linearize(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
    auto res = df(std::vector<cs::DM>{to_cs(x), to_cs(u)});
    A = to_eig(res[0]);
    B = to_eig(res[1]);
}

IterativeLQR::Cost::Cost(int nx, int nu)
{
    Q.setZero(nx, nx);
    R.setZero(nu, nu);
    P.setZero(nu, nx);
    q.setZero(nx);
    r.setZero(nu);
}

void IterativeLQR::Cost::setCost(const casadi::Function &cost)
{
    l = cost;
    dl = l.factory("dl", {"x", "u"}, {"jac:l:x", "jac:l:u"});
    ddl = dl.factory("ddl", {"x", "u"}, {"jac:jac_l_x:x", "jac:jac_l_u:u", "jac:jac_l_u:x"});

    // tbd: do something with this
    bool is_quadratic = ddl.jacobian().nnz_out() == 0;
}

void IterativeLQR::Cost::quadratize(const Eigen::VectorXd &x, const Eigen::VectorXd &u)
{
    auto d_res = dl(std::vector<cs::DM>{to_cs(x), to_cs(u)});
    q = to_eig(d_res[0]).row(0);
    r = to_eig(d_res[1]).row(0);

    auto dd_res = ddl(std::vector<cs::DM>{to_cs(x), to_cs(u)});
    Q = to_eig(dd_res[0]);
    R = to_eig(dd_res[1]);
    P = to_eig(dd_res[2]);
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

WrappedFunction::WrappedFunction(casadi::Function f):
    _f(f)
{
    // resize work vectors
    _iw.assign(_f.sz_iw(), 0);
    _dw.assign(_f.sz_w(), 0.);

    // resize input buffers
    _in_buf.assign(_f.n_in(), nullptr);

    // create memory for output data
    for(int i = 0; i < _f.n_out(); i++)
    {
        const auto& sp = _f.sparsity_out(i);

        // allocate memory for all nonzero elements of this output
        _out_data.emplace_back(sp.nnz(), 0.);

        // push the allocated buffer address to a vector
        _out_buf.push_back(_out_data.back().data());

        // allocate a zero dense matrix to store the output
        _out_matrix.emplace_back(Eigen::MatrixXd::Zero(sp.size1(), sp.size2()));
    }
}

void WrappedFunction::setInput(int i, Eigen::Ref<const Eigen::VectorXd> xi)
{
    _in_buf[i] = xi.data();
}

void WrappedFunction::call()
{
    // call function (allocation-free)
    _f(_in_buf.data(), _out_buf.data(), _iw.data(), _dw.data(), 0);

    // copy all outputs to dense matrices
    for(int i = 0; i < _f.n_out(); i++)
    {
        csc_to_matrix(_f.sparsity_out(i),
                      _out_data[i],
                      _out_matrix[i]);
    }
}

const Eigen::MatrixXd& WrappedFunction::getOutput(int i) const
{
    return _out_matrix[i];
}

casadi::Function &WrappedFunction::function()
{
    return _f;
}

void WrappedFunction::csc_to_matrix(const casadi::Sparsity& sp,
                                    const std::vector<double>& data,
                                    Eigen::MatrixXd& matrix)
{
    // if dense output, do copy assignment which should be
    // faster
    if(sp.is_dense())
    {
        matrix = Eigen::MatrixXd::Map(data.data(),
                                      matrix.rows(),
                                      matrix.cols());
        return;
    }

    int col_j = 0;
    for(int k = 0; k < sp.nnz(); k++)
    {
        // current elem row index
        int row_i = sp.row(k);

        // update current elem col index
        if(k == sp.colind(col_j + 1))
        {
            col_j++;
        }

        // copy data
        matrix(row_i, col_j) =  data[k];
    }
}
