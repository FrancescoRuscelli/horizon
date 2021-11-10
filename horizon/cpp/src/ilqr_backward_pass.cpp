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
            std::cout << "increasing reg at k = " << i << ", hxx_reg = " << _hxx_reg << "\n";
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
    TIC(value_fn_inner);
    auto& value = _value[i];
    auto& S = value.S;
    auto& s = value.s;

    S.noalias() = tmp.Hxx + Lz.transpose()*(tmp.Huu*Lz + tmp.Hux) + tmp.Hux.transpose()*Lz;
    S = 0.5*(S + S.transpose());  // note: symmetrize
    s.noalias() = tmp.hx + tmp.Hux.transpose()*lz + Lz.transpose()*(tmp.hu + tmp.Huu*lz);
    TOC(value_fn_inner);

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

    // number of constraints
    int nc = _constraint_to_go->dim();
    res.nc = nc;

    // compute quadritized free value function (before constraints)
    // note: this is needed by the compute_constrained_input routine!
    tmp.huf.noalias() = r + B.transpose()*(snext + Snext*d);
    tmp.Huuf.noalias() = R + B.transpose()*Snext*B;
    // tmp.Huxf.noalias() = P + B.transpose()*Snext*A;

    // compute constraint-consistent input
    compute_constrained_input(tmp, res);

    // nullspace left after constraints
    int ns_dim = Bz.cols();

    // modified cost and dynamics due to uc = uc(x, z)
    // note: our new control input will be z!
    TIC(constraint_modified_dynamics_inner);
    tmp.Ac.noalias() = A + B*Lc;
    tmp.Bc.noalias() = B*Bz;
    tmp.dc.noalias() = d + B*lc;

    tmp.qc.noalias() = q + Lc.transpose()*(r + R*lc) + P.transpose()*lc;
    tmp.rc.noalias() = Bz.transpose()*(r + R*lc);
    tmp.Qc.noalias() = Q + Lc.transpose()*R*Lc + Lc.transpose()*P + P.transpose()*Lc;
    tmp.Qc = 0.5*(tmp.Qc + tmp.Qc.transpose());
    tmp.Rc.noalias() = Bz.transpose()*R*Bz;
    tmp.Pc.noalias() = Bz.transpose()*(P + R*Lc);
    TOC(constraint_modified_dynamics_inner);

    // return
    ConstrainedDynamics cd = {tmp.Ac, tmp.Bc, tmp.dc};
    ConstrainedCost cc = {tmp.Qc, tmp.Rc, tmp.Pc, tmp.qc, tmp.rc};
    return std::make_tuple(ns_dim, cd, cc);

}

void IterativeLQR::compute_constrained_input(Temporaries& tmp, BackwardPassResult& res)
{
    if(_decomp_type == Qr)
    {
        TIC(compute_constrained_input_qr_inner);
        compute_constrained_input_qr(tmp, res);
    }
    else if(_decomp_type == Svd)
    {
        TIC(compute_constrained_input_svd_inner);
        compute_constrained_input_svd(tmp, res);
    }
    else
    {
        throw std::invalid_argument("invalid decomposition");
    }
}

void IterativeLQR::compute_constrained_input_svd(Temporaries& tmp, BackwardPassResult& res)
{
    auto C = _constraint_to_go->C();
    auto D = _constraint_to_go->D();
    auto h = _constraint_to_go->h();

    // some shorthands
    auto& svd = tmp.svd;
    auto& rotC = tmp.rotC;
    auto& roth = tmp.roth;
    auto& lc = tmp.lc;
    auto& Lc = tmp.Lc;
    auto& Bz = tmp.Bz;

    // svd of input matrix
    const double sv_ratio_thr = _svd_threshold;
    THROW_NAN(D);
    TIC(constraint_svd_inner);
    svd.compute(D, Eigen::ComputeFullU|Eigen::ComputeFullV);
    TOC(constraint_svd_inner);
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
    TIC(constraint_input_inner);
    lc.noalias() = -V.leftCols(rank) * roth.head(rank).cwiseQuotient(sv.head(rank));
    Lc.noalias() = -V.leftCols(rank) * sv.head(rank).cwiseInverse().asDiagonal() * rotC.topRows(rank);
    Bz.noalias() = V.rightCols(ns_dim);
    TOC(constraint_input_inner);

    // compute lagrangian multipliers corresponding to the
    // satisfied component of the constraints, i.e.
    // lam = Gu*u + Gx*x + g, where:
    //  *) Gu = -U[:, :r]*sigma^-1*V[:, :r]'*Huu
    //  *) Gx = -U[:, :r]*sigma^-1*V[:, :r]'*Hux
    //  *) g = -U[:, :r]*sigma^-1*V[:, :r]'*hu
    // note: we only compute the g and Gu term (assume dx = 0)
    TIC(constraint_lagmul_inner);
    tmp.UrSinvVrT.noalias() = U.leftCols(rank)*sv.head(rank).cwiseInverse().asDiagonal()*V.leftCols(rank).transpose();
    res.glam = -tmp.UrSinvVrT*tmp.huf;
    res.Gu = -tmp.UrSinvVrT*tmp.Huuf;
    // res.Gx = -tmp.UrSinvVrT*tmp.Huxf;
    TOC(constraint_lagmul_inner);

    // remove satisfied constraints from constraint to go
    int nc = _constraint_to_go->dim();
    _constraint_to_go->set(rotC.bottomRows(nc - rank),
                           roth.tail(nc - rank));
}


void IterativeLQR::compute_constrained_input_qr(Temporaries &tmp, BackwardPassResult &res)
{
    auto C = _constraint_to_go->C();
    auto D = _constraint_to_go->D();
    auto h = _constraint_to_go->h();

    THROW_NAN(C);
    THROW_NAN(D);
    THROW_NAN(h);

    auto& qr = tmp.qr;
    auto& lc = tmp.lc;
    auto& Lc = tmp.Lc;
    auto& Bz = tmp.Bz;

    // D is fat (we can satisfy all constraints unless rank deficient, and there is
    // a nullspace control input to further minimize the cost)
    if(D.rows() < D.cols())
    {
        // D' = QR
        TIC(constraint_qr_inner);
        qr.compute(D.transpose());
        TOC(constraint_qr_inner);
        Eigen::MatrixXd r = qr.matrixQR().triangularView<Eigen::Upper>();
        THROW_NAN(r);

        // rank estimate
        int rank = D.rows();

        for(int i = D.rows()-1; i >= 0; --i)
        {
            if(std::fabs(r(i, i)) < 1e-9)
            {
                --rank;
            }
            else
            {
                break;
            }
        }

        // rank deficient case?
        int rank_def = D.rows() - rank;

        // nullspace left after constraint
        int ns_dim = D.cols() - rank;

        // r = [r1; 0] =  [r11, r12; 0]
        Eigen::MatrixXd r1 = r.topRows(rank);
        Eigen::MatrixXd r11 = r1.leftCols(rank);
        Eigen::MatrixXd r12 = r1.rightCols(rank_def);

        // q = [q1, q2]
        TIC(constraint_qr_get_q_inner);
        Eigen::MatrixXd q = qr.householderQ();
        TOC(constraint_qr_get_q_inner);
        Eigen::MatrixXd q1 = q.leftCols(rank);
        Eigen::MatrixXd q2 = q.rightCols(ns_dim);
        THROW_NAN(q);

        // we must permute C and h in order to comply
        // with the qr column-pivoting
        Eigen::MatrixXd Cp = qr.colsPermutation().transpose()*C;
        Eigen::VectorXd hp = qr.colsPermutation().transpose()*h;

        // C = [C1; C2], h = [h1; h2]
        auto C1 = Cp.topRows(rank);
        auto C2 = Cp.bottomRows(rank_def);
        auto h1 = hp.head(rank);
        auto h2 = hp.tail(rank_def);

        // r11^-T
        TIC(constraint_r11_inv_T_inner);
        Eigen::MatrixXd r11_t_inv;
        r11_t_inv.setIdentity(rank, rank);
        r11.triangularView<Eigen::Upper>().solveInPlace(r11_t_inv);
        r11_t_inv.transposeInPlace();
        TOC(constraint_r11_inv_T_inner);
        THROW_NAN(r11_t_inv);

        // q1*r1^-T
        TIC(constraint_input_inner);
        Eigen::MatrixXd q1_r11_t_inv;
        q1_r11_t_inv.noalias() = q1*r11_t_inv;

        // compute input
        lc.noalias() = -q1_r11_t_inv*h1;
        Lc.noalias() = -q1_r11_t_inv*C1;
        Bz = q2;
        TOC(constraint_input_inner);

        // compute lag mul
        TIC(constraint_lagmul_inner);
        res.Gu.noalias() = -q1_r11_t_inv.transpose()*tmp.Huuf;
        res.glam.noalias() = -q1_r11_t_inv.transpose()*tmp.huf;
        TOC(constraint_lagmul_inner);

        // compute unsatisfied constraint portion
        Eigen::MatrixXd Cu, r12_t_r11_t_inv;
        Eigen::VectorXd hu;
        r12_t_r11_t_inv.noalias() = r12.transpose()*r11_t_inv;
        Cu.noalias() = C2 - r12_t_r11_t_inv*C1;
        hu.noalias() = h2 - r12_t_r11_t_inv*h1;

        // set unsatisfied constraints to current constraint to go
        _constraint_to_go->set(Cu, hu);
    }
    // D is tall, we can satisfy at most $rank constraints and the rest
    // must be propagated backwards in time
    else
    {
        // D = QR = Q1*R1
        TIC(constraint_qr_inner);
        qr.compute(D);
        TOC(constraint_qr_inner);

        // rank estimate
        int rank = D.cols();
        Eigen::MatrixXd  r = qr.matrixQR().triangularView<Eigen::Upper>();
        THROW_NAN(r);

        for(int i = D.cols()-1; i >= 0; --i)
        {
            if(std::fabs(r(i, i)) < 1e-9)
            {
                --rank;
            }
            else
            {
                break;
            }
        }

        // rank deficient case creates a nullspace
        int ns_dim = D.cols() - rank;

        // unsatisfied constraints
        int nc_left = D.rows() - rank;

        // r = [r1; 0]
        Eigen::MatrixXd r1 = r.topRows(rank);
        Eigen::MatrixXd r11 = r1.leftCols(rank);
        Eigen::MatrixXd r12 = r1.rightCols(ns_dim);

        // q = [q1, q2]
        TIC(constraint_qr_get_q_inner);
        Eigen::MatrixXd q = qr.householderQ();
        TOC(constraint_qr_get_q_inner);
        Eigen::MatrixXd q1 = q.leftCols(rank);
        Eigen::MatrixXd q2 = q.rightCols(nc_left);
        THROW_NAN(q);

        // r1^-1
        TIC(constraint_r11_inv_inner);
        Eigen::MatrixXd r11_inv;
        r11_inv.setIdentity(rank, rank);
        r11.triangularView<Eigen::Upper>().solveInPlace(r11_inv);
        TOC(constraint_r11_inv_inner);
        THROW_NAN(r11_inv);

        // r1^-1*q1^T
        TIC(constraint_input_inner);
        Eigen::MatrixXd r11_inv_q1_t;
        r11_inv_q1_t.noalias() = r11_inv*q1.transpose();

        // compute input
        lc.resize(_nu);
        lc.head(rank).noalias() = -r11_inv_q1_t*h;
        lc.array().tail(ns_dim) = 0;
        Lc.resize(_nu, _nx);
        Lc.topRows(rank).noalias() = -r11_inv_q1_t*C;
        Lc.array().bottomRows(ns_dim) = 0;
        Bz.resize(_nu, ns_dim);
        Bz.topRows(rank).noalias() = -r11_inv*r12;
        Bz.bottomRows(ns_dim).setIdentity(ns_dim, ns_dim);

        // apply permutation to input in order to comply
        // with qr column pivoting
        lc = qr.colsPermutation()*lc;
        Lc = qr.colsPermutation()*Lc;
        Bz = qr.colsPermutation()*Bz;
        TOC(constraint_input_inner);

        // compute lagrangian multiplier
        TIC(constraint_lagmul_inner);
        res.Gu.noalias() = -r11_inv.transpose()*tmp.Huuf.topRows(rank);
        res.glam.noalias() = -r11_inv.transpose()*tmp.huf.head(rank);
        TOC(constraint_lagmul_inner);

        // set unsatisfied constraints to current constraint to go
        _constraint_to_go->set(q2.transpose()*C, q2.transpose()*h);

    }

}



