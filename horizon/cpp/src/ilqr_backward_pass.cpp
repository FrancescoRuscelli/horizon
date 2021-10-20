#include "ilqr_impl.h"

struct HessianIndefinite : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

void IterativeLQR::backward_pass()
{
    TIC(backward_pass);

    // initialize backward recursion from final cost..
    _value.back().S = _cost.back().Q();
    _value.back().s = _cost.back().q();

    // ..and constraint
    _constraint_to_go->set(_constraint.back());

    // backward pass
    bool reg_needed = false;
    int i = _N - 1;
    while(i >= 0)
    {
        if(i == _N)
        {
            // if we got here (because of an HessianIndefinite exception),
            // we need to regularize the final cost!
            _value.back().S.diagonal().array() += _hxx_reg;
            --i;
            continue;
        }

        try
        {
            backward_pass_iter(i);
            --i;
        }
        catch(HessianIndefinite&)
        {
            increase_regularization();
            reg_needed = true;
            ++i;
        }
    }

    if(!reg_needed)
    {
        reduce_regularization();
    }
}

void IterativeLQR::backward_pass_iter(int i)
{
    TIC(backward_pass_inner);

    // constraint handling
    auto [nz, cdyn, ccost] = handle_constraints(i);

    const bool has_constraints = (nz != _nu);

    // note: after handling constraints, we're actually optimizing an
    // auxiliary input z, where the original input u = lc + Lc*x + Lz*z

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

    // ..value function
    const auto& value_next = _value[i+1];
    const auto& Snext = value_next.S;
    const auto& snext = value_next.s;

    THROW_NAN(Snext);
    THROW_NAN(snext);

    // ..workspace
    auto& tmp = _tmp[i];

    // mapping to original input u
    const auto& lc = tmp.lc;
    const auto& Lc = tmp.Lc;
    const auto& Bz = tmp.Bz;

    // components of next node's value function (as a function of
    // current state and control via the dynamics)
    // note: first compute state-only components, since after constraints
    // there might be no input dof to optimize at all!
    tmp.s_plus_S_d.noalias() = snext + Snext*d;
    tmp.S_A.noalias() = Snext*A;

    tmp.hx.noalias() = q + A.transpose()*tmp.s_plus_S_d;
    tmp.Hxx.noalias() = Q + A.transpose()*tmp.S_A;
    tmp.Hxx.diagonal().array() += _hxx_reg;

//    double eig_min_Q = Q.eigenvalues().real().minCoeff();
//    tmp.Hxx.diagonal().array() -= std::min(eig_min_Q, 0.0)*2.0;


    // handle case where nz = 0, i.e. no nullspace left after constraints
    if(nz == 0)
    {
        // save solution
        auto& res = _bp_res[i];
        res.Lu = Lc;
        res.lu = lc;
        res.Lz.setZero(0, _nx);
        res.lz.setZero(0);

        // save multipliers

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
    TIC(llt_inner);
    tmp.llt.compute(tmp.Huu);
    tmp.llt.solveInPlace(tmp.huHux);
    if(tmp.llt.info() != Eigen::ComputationInfo::Success)
    {
        throw HessianIndefinite("backward pass error: hessian not positive");
    }
    TOC(llt_inner);

    THROW_NAN(tmp.huHux);

    // save solution
    auto& res = _bp_res[i];
    auto& Lz = res.Lz;
    auto& lz = res.lz;
    Lz = -tmp.huHux.rightCols(_nx);
    lz = -tmp.huHux.col(0);

    // save optimal value function
    auto& value = _value[i];
    auto& S = value.S;
    auto& s = value.s;

    S.noalias() = tmp.Hxx + Lz.transpose()*(tmp.Huu*Lz + tmp.Hux) + tmp.Hux.transpose()*Lz;
    S = 0.5*(S + S.transpose());  // note: symmetrize
    s.noalias() = tmp.hx + tmp.Hux.transpose()*lz + Lz.transpose()*(tmp.hu + tmp.Huu*lz);

    // map to original input u
    if(has_constraints)
    {
        res.lu.noalias() = lc + Bz*lz;
        res.Lu.noalias() = Lc + Bz*Lz;
    }
    else
    {
        res.lu = lz;
        res.Lu = Lz;
        tmp.huf = tmp.hu;
    }

}

void IterativeLQR::increase_regularization()
{
    if(_hxx_reg < 1e-6)
    {
        _hxx_reg = 1.0;
    }

    _hxx_reg *= _hxx_reg_growth_factor;

    if(_hxx_reg > 1e12)
    {
        throw std::runtime_error("maximum regularization exceeded");
    }
}

void IterativeLQR::reduce_regularization()
{
    _hxx_reg /= std::sqrt(_hxx_reg_growth_factor);
}

IterativeLQR::HandleConstraintsRetType IterativeLQR::handle_constraints(int i)
{
    TIC(handle_constraints_inner);

    // some shorthands for..

    // ..value function
    const auto& value_next = _value[i+1];
    const auto& Snext = value_next.S;
    const auto& snext = value_next.s;

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
    const auto& d = dyn.d;  // note: has been computed during linearization phase

    // ..workspace
    auto& tmp = _tmp[i];
    auto& svd = tmp.svd;
    auto& rotC = tmp.rotC;
    auto& roth = tmp.roth;
    auto& lc = tmp.lc;
    auto& Lc = tmp.Lc;
    auto& Bz = tmp.Bz;

    // ..backward pass result
    auto& res = _bp_res[i];

    // no constraint to handle, do nothing
    if(_constraint_to_go->dim() == 0 &&
            !_constraint[i].is_valid())
    {
        ConstrainedDynamics cd = {A, B, d};
        ConstrainedCost cc = {Q, R, P, q, r};
        res.nc = 0;
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
    res.nc = nc;

    // svd of input matrix
    const double sv_ratio_thr = _svd_threshold;
    THROW_NAN(D);
    TIC(svd_inner);
    svd.compute(D, Eigen::ComputeFullU|Eigen::ComputeFullV);
    TOC(svd_inner);
    const auto& U = svd.matrixU();
    const auto& V = svd.matrixV();
    const auto& sv = svd.singularValues();
    THROW_NAN(svd.singularValues());
    THROW_NAN(svd.matrixU());
    THROW_NAN(svd.matrixV());
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
    Bz.noalias() = V.rightCols(ns_dim);

    // compute quadritized free value function (before constraints)
    tmp.huf.noalias() = r + B.transpose()*(snext + Snext*d);
    tmp.Huuf.noalias() = R + B.transpose()*Snext*B;
    // tmp.Huxf.noalias() = P + B.transpose()*Snext*A;

    // compute lagrangian multipliers corresponding to the
    // satisfied component of the constraints, i.e.
    // lam = Gu*u + Gx*x + g, where:
    //  *) Gu = -U[:, :r]*sigma^-1*V[:, :r]'*Huu
    //  *) Gx = -U[:, :r]*sigma^-1*V[:, :r]'*Hux
    //  *) g = -U[:, :r]*sigma^-1*V[:, :r]'*hu
    // note: we only compute the g and Gu term (assume dx = 0)
    tmp.UrSinvVrT.noalias() = U.leftCols(rank)*sv.head(rank).cwiseInverse().asDiagonal()*V.leftCols(rank).transpose();
    res.glam = -tmp.UrSinvVrT*tmp.huf;
    res.Gu = -tmp.UrSinvVrT*tmp.Huuf;
    // res.Gx = -tmp.UrSinvVrT*tmp.Huxf;

    // remove satisfied constraints from constraint to go
    _constraint_to_go->set(rotC.bottomRows(nc - rank),
                           roth.tail(nc - rank));

    // modified cost and dynamics due to uc = uc(x, z)
    // note: our new control input will be z!
    tmp.Ac.noalias() = A + B*Lc;
    tmp.Bc.noalias() = B*Bz;
    tmp.dc.noalias() = d + B*lc;

    tmp.qc.noalias() = q + Lc.transpose()*(r + R*lc) + P.transpose()*lc;
    tmp.rc.noalias() = Bz.transpose()*(r + R*lc);
    tmp.Qc.noalias() = Q + Lc.transpose()*R*Lc + Lc.transpose()*P + P.transpose()*Lc;
    tmp.Qc = 0.5*(tmp.Qc + tmp.Qc.transpose());
    tmp.Rc.noalias() = Bz.transpose()*R*Bz;
    tmp.Pc.noalias() = Bz.transpose()*(P + R*Lc);

    // return
    ConstrainedDynamics cd = {tmp.Ac, tmp.Bc, tmp.dc};
    ConstrainedCost cc = {tmp.Qc, tmp.Rc, tmp.Pc, tmp.qc, tmp.rc};
    return std::make_tuple(ns_dim, cd, cc);

}



