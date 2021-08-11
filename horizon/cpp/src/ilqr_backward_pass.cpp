#include "ilqr_impl.h"

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
    TIC(backward_pass_inner);

    // some shorthands..

    // ..value function
    const auto& value_next = _value[i+1];
    const auto& Snext = value_next.S;
    const auto& snext = value_next.s;

    // ..defect
    const auto& d = _dyn[i].d;

    // ..workspace
    auto& tmp = _tmp[i];

    // note: compute s + S*d here since handle_constraints needs it
    tmp.s_plus_S_d.noalias() = snext + Snext*d;

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



    // mapping to original input u
    const auto& lc = tmp.lc;
    const auto& Lc = tmp.Lc;
    const auto& Bz = tmp.Bz;

    // components of next node's value function (as a function of
    // current state and control via the dynamics)
    // note: first compute state-only components, since after constraints
    // there might be no input dof to optimize at all!
    tmp.S_A.noalias() = Snext*A;

    tmp.hx.noalias() = q + A.transpose()*tmp.s_plus_S_d;
    tmp.Hxx.noalias() = Q + A.transpose()*tmp.S_A;


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
    tmp.llt.compute(tmp.Huu);
    tmp.llt.solveInPlace(tmp.huHux);

    // todo: check solution for nan, unable to solve, etc

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

    S.noalias() = tmp.Hxx - Lz.transpose()*tmp.Huu*Lz;
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
    }

}

IterativeLQR::HandleConstraintsRetType IterativeLQR::handle_constraints(int i)
{
    TIC(handle_constraints_inner);

    // some shorthands for..

    // ..value function
    const auto& value_next = _value[i+1];
    const auto& Snext = value_next.S;

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
    Bz.noalias() = V.rightCols(ns_dim);

    // compute quadritized free value function (before constraints)
    tmp.huf.noalias() = r + B.transpose()*tmp.s_plus_S_d;
    tmp.Huuf.noalias() = R + B.transpose()*Snext*B;
    tmp.Huxf.noalias() = P + B.transpose()*tmp.S_A;

    // compute lagrangian multipliers corresponding to the
    // satisfied component of the constraints, i.e.
    // lam = Gu*u + Gx*x + g, where:
    //  *) Gu = -U[:, :r]*sigma^-1*V[:, :r]'*Huu
    //  *) Gx = -U[:, :r]*sigma^-1*V[:, :r]'*Hux
    //  *) g = -U[:, :r]*sigma^-1*V[:, :r]'*hu
    tmp.UrSinvVrT.noalias() = U.leftCols(rank)*sv.head(rank).cwiseInverse().asDiagonal()*V.leftCols(rank).transpose();
    res.Gu = -tmp.UrSinvVrT*tmp.Huuf;
    res.Gx = -tmp.UrSinvVrT*tmp.Huxf;
    res.glam = -tmp.UrSinvVrT*tmp.huf;

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
    tmp.Rc.noalias() = Bz.transpose()*R*Bz;
    tmp.Pc.noalias() = Bz.transpose()*(P + R*Lc);

    // return
    ConstrainedDynamics cd = {tmp.Ac, tmp.Bc, tmp.dc};
    ConstrainedCost cc = {tmp.Qc, tmp.Rc, tmp.Pc, tmp.qc, tmp.rc};
    return std::make_tuple(ns_dim, cd, cc);

}
