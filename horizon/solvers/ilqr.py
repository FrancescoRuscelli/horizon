import casadi as cs
import numpy as np
from scipy.special import comb
from horizon.utils import utils



class iterativeLQR:
    """
    The iterativeLQR class solves a nonlinear, (equality)constrained iLQR problem for a given
     - system dynamics (continuous time)
     - intermediate cost l(x,u)
     - final cost lf(x)

     Equality constraints are treated by mean of projection: cost function is projected onto equality constraints.
    """

    class LinearDynamics:

        def __init__(self, nx, nu):
            # type: (int, int) -> None

            self.A = np.zeros((nx, nx))
            self.B = np.zeros((nx, nu))
            self.Fxx = np.zeros((nx*nx, nx))
            self.Fuu = np.zeros((nx*nu, nu))
            self.Fux = np.zeros((nx*nu, nx))

        def __repr__(self):
            return self.__dict__.__repr__()

    class LinearConstraint:

        def __init__(self, nx, nu, nc):
            # type: (int, int, int) -> None

            self.C = np.zeros((nc, nx))
            self.D = np.zeros((nc, nu))
            self.g = np.zeros(nc)

        def __repr__(self):
            return self.__dict__.__repr__()

    class QuadraticCost:

        def __init__(self, nx, nu):
            # type: (int, int) -> None
            self.qx = np.zeros(nx)
            self.Qxx = np.zeros((nx, nx))
            self.qu = np.zeros(nu)
            self.Quu = np.zeros((nu, nu))
            self.Qxu = np.zeros((nx, nu))

        def __repr__(self):
            return self.__dict__.__repr__()

    def __init__(self,
                 x, u, xdot,
                 dt, N,
                 intermediate_cost, final_cost,
                 intermediate_constraints=dict(),
                 final_constraint=None,
                 sym_t=cs.SX,
                 opt=dict()): # contains options for the solver: {"lambda" : float}
        # type: (cs.SX, cs.SX, cs.SX, float, int, cs.SX, cs.SX, Dict, cs.SX, cs.SX, Dict) -> None

        """
        Constructor
        :param x: state variable
        :param u: control variable
        :param xdot: continuous-time dynamics -> xdot = f(x, u)
        :param dt: discretization step
        :param N: horizon length
        :param intermediate_cost: intermediate cost -> l(x, u)
        :param final_cost: final cost -> lf(x)
        :param intermediate_constraints: dict constr_name -> hi(x, u) = 0
        :param final_constraint: hf(x) = 0
        :param sym_t: casadi symbol type (SX or MX)
        """

        # sym type
        self._sym_t = sym_t

        # state and control dimension
        self._nx = x.size1()
        self._nu = u.size1()

        # discretization & horizon
        self._dt = dt
        self._N = N

        # dynamics
        self._dynamics_ct = cs.Function('dynamics_ct',
                                        {'x': x, 'u': u, 'xdot': xdot},
                                        ['x', 'u'],
                                        ['xdot'])

        # cost terms
        self._diff_inter_cost = cs.Function('intermediate_cost',
                                            {'x': x, 'u': u, 'l': intermediate_cost},
                                            ['x', 'u'],
                                            ['l'])

        self._final_cost = cs.Function('final_cost',
                                       {'x': x, 'u': u, 'lf': final_cost},
                                       ['x', 'u'],
                                       ['lf'])

        d = {'x': x, 'u': u, 'lf': final_cost}
        self._jacobian_lf, _tmp_functions = utils.jac(d, ['x', 'u'], ['lf'])
        dd = dict(list(d.items()) + list(_tmp_functions.items()))
        self._hessian_lf, _ = utils.jac(dd, ['x', 'u'], _tmp_functions.keys())

        # discrete dynamics & intermediate cost
        self._discretize()

        # constraints
        self._constrained = final_constraint != None or len(intermediate_constraints) != 0
        self._final_constraint = None
        self._constraint_to_go = None

        if self._constrained:
            self._inter_constraints = [self.LinearConstraint(self._nx, self._nu, 0) for _ in range(self._N)]

        # final constraint
        if final_constraint is not None:
            self._constraint_to_go = self.LinearConstraint(self._nx, self._nu, 0)

            self._final_constraint = cs.Function('final_constraint',
                                                 {'x': x, 'u': u, 'hf': final_constraint},
                                                 ['x', 'u'],
                                                 ['hf'])

            self._final_constraint_jac, _ = jac({'x': x, 'u': u, 'hf': final_constraint}, ['x', 'u'], ['hf'])


        # intermediate constraints
        intermediate_constr_r_der = []  # list of intermediate constraint r-th order dynamics

        # loop over all defined constraints, fill above lists
        for name, ic in intermediate_constraints.items():

            rel_degree = 0
            hi_derivatives = [ic]  # list of current constraint derivatives

            while True:
                inter_constraint_jac, _ = jac({'x': x, 'u': u, 'h': ic}, ['x', 'u'], ['h'])

                # if constraint jacobian depends on u, break
                if inter_constraint_jac(x=x, u=u)['DhDu'].nnz() > 0:
                    break

                # otherwise, increase relative degree and do time derivative
                rel_degree += 1
                ic = cs.mtimes(inter_constraint_jac(x=x, u=u)['DhDx'], xdot)
                hi_derivatives.append(ic)

            print('constraint "{}" relative degree is {}'.format(name, rel_degree))
            rand_x = np.random.standard_normal(self._nx)
            rand_u = np.random.standard_normal(self._nu)
            r = np.linalg.matrix_rank(inter_constraint_jac(x=rand_x, u=rand_u)['DhDu'].toarray())
            print('constraint "{}" rank at random (x, u) is {} vs dim(u) = {}'.format(name, r, self._nu))

            hi_dynamics = 0

            self._constr_lambda = 10.0
            if "lambda" in opt:
                self._constr_lambda = opt["lambda"]
            for i in range(rel_degree+1):
                hi_dynamics += comb(rel_degree, i) * hi_derivatives[i] * self._constr_lambda**(rel_degree-i)

            intermediate_constr_r_der.append(hi_dynamics)


        self._inter_constr = cs.Function('intermediate_constraint',
                                         {'x': x, 'u': u, 'h': cs.vertcat(*intermediate_constr_r_der)},
                                         ['x', 'u'],
                                         ['h'])

        self._inter_constr_jac, _ = utils.jac({'x': x, 'u': u, 'h': cs.vertcat(*intermediate_constr_r_der)}, ['x', 'u'], ['h'])

        self._has_inter_constr = self._inter_constr.size1_out('h') > 0

        # initalization of all internal structures
        self._state_trj = [np.zeros(self._nx) for _ in range(self._N + 1)]
        self._ctrl_trj  = [np.zeros(self._nu) for _ in range(self._N)]
        self._inter_constr_trj = [np.zeros(0) for _ in range(self._N)]
        self._lin_dynamics = [self.LinearDynamics(self._nx, self._nu) for _ in range(self._N)]
        self._inter_quad_cost = [self.QuadraticCost(self._nx, self._nu) for _ in range(self._N)]
        self._final_quad_cost = self.QuadraticCost(self._nx, 0)
        self._cost_to_go = [self.QuadraticCost(self._nx, self._nu) for _ in range(self._N)]
        self._value_function = [self.QuadraticCost(self._nx, self._nu) for _ in range(self._N)]

        self._fb_gain = [np.zeros((self._nu, self._nx)) for _ in range(self._N)]
        self._ff_u = [np.zeros(self._nu) for _ in range(self._N)]
        self._defect = [np.zeros(self._nx) for _ in range(self._N)]

        self._defect_norm = []
        self._du_norm = []
        self._dx_norm = []
        self._dcost = []

        self._use_second_order_dynamics = False
        self._use_single_shooting_state_update = False
        self._verbose = False

        #Conditioning number for the KKY inversion
        self._kkt_rcond = 1e-6
        self._svd_thr = 1e-4

    @staticmethod
    def _make_jit_function(f):
        # type: (cs.Function) -> cs.external
        """
        Compiles casadi function into a shared object and return it
        :return:
        """

        import filecmp
        import os

        gen_code_path = 'ilqr_generated_{}.c'.format(f.name())
        f.generate(gen_code_path)

        gen_lib_path = 'ilqr_generated_{}.so'.format(f.name())
        gcc_cmd = 'gcc {} -shared -fPIC -O3 -o {}'.format(gen_code_path, gen_lib_path)

        if os.system(gcc_cmd) != 0:
            raise SystemError('Unable to compile function "{}"'.format(f.name()))

        jit_f = cs.external(f.name(), './' + gen_lib_path)

        os.remove(gen_code_path)
        os.remove(gen_lib_path)

        return jit_f

    def _discretize(self):
        """
        Compute discretized dynamics in the form of _F (nonlinear state transition function) and
        _jacobian_F (its jacobian)
        :return: None
        """

        x = self._sym_t.sym('x', self._nx)
        u = self._sym_t.sym('u', self._nu)

        dae = {'x': x,
               'p': u,
               'ode': self._dynamics_ct(x, u),
               'quad': self._diff_inter_cost(x, u)}

        # self._F = cs.integrator('F', 'rk', dae, {'t0': 0, 'tf': self._dt})
        self._F = cs.Function('F',
                              {'x0': x, 'p': u,
                               'xf': x + self._dt * self._dynamics_ct(x, u),
                               'qf': self._dt * self._diff_inter_cost(x, u)
                               },
                              ['x0', 'p'],
                              ['xf', 'qf'])

        # self._F = integrator.RK4(dae, {'tf': self._dt}, 'SX')
        d = {'x0': x, 'p': u, 'xf': x + self._dt * self._dynamics_ct(x, u), 'qf': self._dt * self._diff_inter_cost(x, u)}
        self._jacobian_F, _tmp_functions = utils.jac(d, ['x0', 'p'], ['xf', 'qf'])
        dd = dict(list(d.items()) + list(_tmp_functions.items()))
        self._hessian_F, _ = utils.jac(dd, ['x0', 'p'], _tmp_functions.keys())

    def _linearize_quadratize(self):
        """
        Compute quadratic approximations to cost functions about the current state and control trajectories
        :return: None
        """

        jl_value = self._jacobian_lf(x=self._state_trj[-1])
        hl_value = self._hessian_lf(x=self._state_trj[-1])

        self._final_quad_cost.qx = jl_value['DlfDx'].toarray().flatten()
        self._final_quad_cost.Qxx = hl_value['DDlfDxDx'].toarray()

        for i in range(self._N):

            jode_value = self._jacobian_F(x0=self._state_trj[i],
                                          p=self._ctrl_trj[i])

            hode_value = self._hessian_F(x0=self._state_trj[i],
                                         p=self._ctrl_trj[i])

            self._inter_quad_cost[i].qu = jode_value['DqfDp'].toarray().flatten()
            self._inter_quad_cost[i].qx = jode_value['DqfDx0'].toarray().flatten()
            self._inter_quad_cost[i].Quu = hode_value['DDqfDpDp'].toarray()
            self._inter_quad_cost[i].Qxx = hode_value['DDqfDx0Dx0'].toarray()
            self._inter_quad_cost[i].Qxu = hode_value['DDqfDx0Dp'].toarray()

            self._lin_dynamics[i].A = jode_value['DxfDx0'].toarray()
            self._lin_dynamics[i].B = jode_value['DxfDp'].toarray()

            if self._use_second_order_dynamics:
                for j in range(self._nx):
                    nx = self._nx
                    nu = self._nu
                    self._lin_dynamics[i].Fxx[j*nx:(j+1)*nx, :] = hode_value['DDxfDx0Dx0'].toarray()[j::nx, :]
                    self._lin_dynamics[i].Fuu[j*nu:(j+1)*nu, :] = hode_value['DDxfDpDp'].toarray()[j::nx, :]
                    self._lin_dynamics[i].Fux[j*nu:(j+1)*nu, :] = hode_value['DDxfDpDx0'].toarray()[j::nx, :]

            if self._constrained:

                jconstr_value = self._inter_constr_jac(x=self._state_trj[i],
                                                       u=self._ctrl_trj[i])

                self._inter_constraints[i].C = jconstr_value['DhDx'].toarray()
                self._inter_constraints[i].D = jconstr_value['DhDu'].toarray()
                self._inter_constraints[i].g = self._inter_constr(x=self._state_trj[i],
                                                                  u=self._ctrl_trj[i])['h'].toarray().flatten()

        if self._final_constraint is not None:

            jgf_value = self._final_constraint_jac(x=self._state_trj[-1])['DhfDx']
            nc = self._final_constraint.size1_out('hf')
            self._constraint_to_go = self.LinearConstraint(self._nx, self._nu, nc)
            self._constraint_to_go.C = jgf_value.toarray()
            self._constraint_to_go.D = np.zeros((nc, self._nu))
            self._constraint_to_go.g = self._final_constraint(x=self._state_trj[-1])['hf'].toarray().flatten()

    def set_kkt_rcond(self, rcond):
        self._kkt_rcond = rcond

    def get_kkt_rcond(self):
        return self._kkt_rcond

    def set_svd_thr(self, svd_thr):
        self._svd_thr = svd_thr

    def get_svd_thr(self):
        return self._svd_thr

    def _backward_pass(self):
        """
        To be implemented
        :return:
        """

        # value function at next time step (prev iteration)
        S = self._final_quad_cost.Qxx
        s = self._final_quad_cost.qx


        for i in reversed(range(self._N)):

            # variable labeling for better convenience
            nx = self._nx
            nu = self._nu
            x_integrated = self._F(x0=self._state_trj[i], p=self._ctrl_trj[i])['xf'].toarray().flatten()
            xnext = self._state_trj[i+1]
            d = x_integrated - xnext
            r = self._inter_quad_cost[i].qu
            q = self._inter_quad_cost[i].qx
            P = self._inter_quad_cost[i].Qxu.T
            R = self._inter_quad_cost[i].Quu
            Q = self._inter_quad_cost[i].Qxx
            A = self._lin_dynamics[i].A
            B = self._lin_dynamics[i].B
            Fxx = self._lin_dynamics[i].Fxx.reshape((nx, nx, nx))
            Fuu = self._lin_dynamics[i].Fuu.reshape((nx, nu, nu))
            Fux = self._lin_dynamics[i].Fux.reshape((nx, nu, nx))

            # intermediate constraints
            C = None
            D = None
            g = None
            if self._has_inter_constr:
                # extract intermediate constraints
                C = self._inter_constraints[i].C
                D = self._inter_constraints[i].D
                g = self._inter_constraints[i].g
                # TODO: second order dynamics?

            # final constraint
            l_ff = np.zeros(self._nu)
            L_fb = np.zeros((self._nu, self._nx))
            Vns = np.eye(self._nu)

            if self._constraint_to_go is not None:

                # back-propagate constraint to go from next time step
                C_to_go = np.matmul(self._constraint_to_go.C, A)
                D_to_go = np.matmul(self._constraint_to_go.C, B)
                g_to_go = self._constraint_to_go.g - np.matmul(self._constraint_to_go.C, d)

                # svd of constraint input matrix
                U, sv, V = np.linalg.svd(D_to_go)
                V = V.T

                # rotated constraint
                rot_g = np.matmul(U.T, g_to_go)
                rot_C = np.matmul(U.T, C_to_go)

                # non-zero singular values
                large_sv = sv > self._svd_thr

                nc = g_to_go.size  # number of currently active constraints
                nsv = len(sv)  # number of singular values
                rank = np.count_nonzero(large_sv)  # constraint input matrix rank

                # singular value inversion
                inv_sv = sv.copy()
                inv_sv[large_sv] = np.reciprocal(sv[large_sv])

                # compute constraint component of control input uc = Lc*x + lc
                l_ff = np.matmul(-V[:, 0:nsv], (inv_sv * rot_g[0:nsv]))
                l_ff.flatten()
                L_fb = np.matmul(-V[:, 0:nsv], np.matmul(np.diag(inv_sv), rot_C[0:nsv, :]))

                # update constraint to go
                left_constraint_dim = nc - rank

                if left_constraint_dim > 0:
                    self._constraint_to_go.C = rot_C[rank:, :]
                    self._constraint_to_go.D = np.zeros((left_constraint_dim, self._nu))
                    self._constraint_to_go.g = rot_g[rank:]

                nullspace_dim = self._nu - rank

                if nullspace_dim == 0:
                    Vns = np.zeros((self._nu, 0))
                else:
                    Vns = V[:, -nullspace_dim:]

                # the constraint induces a modified dynamics via u = Lx + l + Vns*z (z = new control input)
                d = d + np.matmul(B, l_ff)
                A = A + np.matmul(B, L_fb)
                B = np.matmul(B, Vns)

                if self._use_second_order_dynamics:
                    tr_idx = (0, 2, 1)

                    d += np.matmul(0.5 * l_ff, np.matmul(Fuu, l_ff))
                    A += np.matmul(l_ff, np.matmul(Fuu, L_fb)) + np.matmul(l_ff, Fux)
                    B += np.matmul(l_ff, np.matmul(Fuu, Vns))

                    Fxx = Fxx + np.matmul(L_fb.T, (np.matmul(Fuu, L_fb) + Fux)) + np.matmul(Fux.transpose(tr_idx),
                                                                                            L_fb)
                    Fux = np.matmul(Vns.T, (Fux + np.matmul(Fuu, L_fb)))
                    Fuu = np.matmul(Vns.T, np.matmul(Fuu, Vns))

                q = q + np.matmul(L_fb.T, (r + np.matmul(R, l_ff))) + np.matmul(P.T, l_ff)
                Q = Q + np.matmul(L_fb.T, np.matmul(R, L_fb)) + np.matmul(L_fb.T, P) + np.matmul(P.T, L_fb)
                P = np.matmul(Vns.T, (P + np.matmul(R, L_fb)))

                r = np.matmul(Vns.T, (r + np.matmul(R, l_ff)))
                R = np.matmul(Vns.T, np.matmul(R, Vns))

                if left_constraint_dim == 0:
                    self._constraint_to_go = None

                if self._has_inter_constr:
                    g += np.matmul(D, l_ff)
                    C += np.matmul(D, L_fb)
                    D = np.matmul(D, Vns)




            # intermediate quantities
            hx = q + np.matmul(A.T, (s + np.matmul(S, d)))
            hu = r + np.matmul(B.T, (s + np.matmul(S, d)))
            Huu = R + np.matmul(B.T, np.matmul(S, B))
            Hux = P + np.matmul(B.T, np.matmul(S, A))
            Hxx = Q + np.matmul(A.T, np.matmul(S, A))

            if self._use_second_order_dynamics:

                Huu += (np.matmul(Fuu.T, (s + np.matmul(S, d)))).T
                Hux += (np.matmul(Fux.T, (s + np.matmul(S, d)))).T
                Hxx += (np.matmul(Fxx.T, (s + np.matmul(S, d)))).T

            # nullspace projector gain and feedforward computation
            if Huu.shape[1] > 0:
                if D is not None:
                    K = np.vstack([np.hstack([Huu, D.transpose()]), np.hstack([D, np.zeros((D.shape[0], D.shape[0]))])])
                    iK = np.linalg.pinv(K, rcond=self._kkt_rcond)
                    lz = np.matmul(iK, np.vstack([-hu.reshape((hu.size, 1)), -g.reshape((g.size, 1))]))[0:Huu.shape[0],:].reshape(Huu.shape[0])
                    Lz = np.matmul(iK, np.vstack([-Hux, -C]))[0:Huu.shape[0],:]

                    # NULL-SPACE PROJECTOR IMPLEMENTATION
                    # iH = np.linalg.pinv(Huu, rcond=1e-6)  # H^-1
                    #
                    # iHDt = np.matmul(iH, D.transpose()) # H^-1* D'
                    # O = np.linalg.pinv(np.matmul(D, iHDt), rcond=1e-6) # (D*H^-1*D')^-1
                    # pinvD = np.matmul(iHDt, O) # H^-1*D'*(D*H^-1*D')^-1 = D^#
                    #
                    # pDD = np.matmul(pinvD, D) # D^# * D
                    # I = np.eye(*pDD.shape)
                    # NP = np.matmul(iH, (I - pDD)) # H^-1 * (I - D^# * D)
                    #
                    # lz = -np.matmul(pinvD, g) -np.matmul(NP, hu.transpose()) # -D^#*g - H^-1 * (I - D^# * D)*hu'
                    # Lz = -np.matmul(pinvD, C) -np.matmul(NP, Hux)# -D^#*C - H^-1 * (I - D^# * D)*Hux
                else:
                    iH = np.linalg.pinv(Huu, rcond=1e-6)  # H^-1
                    lz = -np.matmul(iH,hu.transpose()) # H^-1 * hu'
                    Lz = -np.matmul(iH,Hux)# H^-1 * Hux




            # overall gain and ffwd including constraint
            l_ff = l_ff + np.matmul(Vns, lz)
            L_fb = L_fb + np.matmul(Vns, Lz)

            # value function update
            s = hx - np.matmul(Lz.T, np.matmul(Huu, lz))
            S = Hxx - np.matmul(Lz.T, np.matmul(Huu, Lz))

            # save gain and ffwd
            self._fb_gain[i] = L_fb.copy()
            self._ff_u[i] = l_ff.copy()

            # save defect (for original dynamics)
            d = x_integrated - xnext
            self._defect[i] = d.copy()

    class PropagateResult:
        def __init__(self):
            self.state_trj = []
            self.ctrl_trj = []
            self.dx_norm = 0.0
            self.du_norm = 0.0
            self.cost = 0.0
            self.inter_constr = []
            self.final_constr = None

    def _forward_pass(self):
        """
        To be implemented
        :return:
        """
        x_old = self._state_trj[:]

        defect_norm = 0
        du_norm = 0
        dx_norm = 0

        for i in range(self._N):

            xnext = self._state_trj[i+1]
            xi_upd = self._state_trj[i]
            ui = self._ctrl_trj[i]
            d = self._defect[i]
            A = self._lin_dynamics[i].A
            B = self._lin_dynamics[i].B
            L = self._fb_gain[i]
            l = self._ff_u[i]
            dx = np.atleast_1d(xi_upd - x_old[i])

            ui_upd = ui + l + np.matmul(L, dx)

            if self._use_single_shooting_state_update:
                xnext_upd = self._F(x0=xi_upd, p=ui_upd)['xf'].toarray().flatten()
            else:
                xnext_upd = xnext + np.matmul((A + np.matmul(B, L)), dx) + np.matmul(B, l) + d

            self._state_trj[i+1] = xnext_upd.copy()
            self._ctrl_trj[i] = ui_upd.copy()

            defect_norm += np.linalg.norm(d, ord=1)
            du_norm += np.linalg.norm(l, ord=1)
            dx_norm += np.linalg.norm(dx, ord=1)
            self._inter_constr_trj[i] = self._inter_constr(x=xi_upd, u=ui_upd)['h'].toarray().flatten()

        self._defect_norm.append(defect_norm)
        self._du_norm.append(du_norm)
        self._dx_norm.append(dx_norm)
        self._dcost.append(self._eval_cost(self._state_trj, self._ctrl_trj))

    def _propagate(self, xtrj, utrj, alpha=1):
        # type: (List[np.array], List[np.array], int) -> PropagateResult

        N = len(utrj)
        ret = self.PropagateResult()

        ret.state_trj = xtrj.copy()
        ret.ctrl_trj = utrj.copy()

        for i in range(N):

            xnext = xtrj[i+1]
            xi = xtrj[i]
            xi_upd = ret.state_trj[i]
            ui = utrj[i]
            d = self._defect[i]
            A = self._lin_dynamics[i].A
            B = self._lin_dynamics[i].B
            L = self._fb_gain[i]
            l = alpha * self._ff_u[i]
            dx = np.atleast_1d(xi_upd - xi)

            ui_upd = ui + l + np.matmul(L, dx)

            if self._use_single_shooting_state_update:
                xnext_upd = self._F(x0=xi_upd, p=ui_upd)['xf'].toarray().flatten()
            else:
                xnext_upd = xnext + np.matmul((A + np.matmul(B, L)), dx) + np.matmul(B, l) + d

            ret.state_trj[i+1] = xnext_upd.copy()
            ret.ctrl_trj[i] = ui_upd.copy()
            ret.dx_norm += np.linalg.norm(dx, ord=1)
            ret.du_norm += np.linalg.norm(ui_upd - ui, ord=1)
            ret.inter_constr.append(self._inter_constr(x=xi_upd, u=ui_upd)['h'].toarray().flatten())

        ret.final_constr = self._final_constraint(x=ret.state_trj[-1])['hf'].toarray().flatten()
        ret.cost = self._eval_cost(ret.state_trj, ret.ctrl_trj)

        return ret

    def _eval_cost(self, x_trj, u_trj):

        cost = 0.0

        for i in range(len(u_trj)):

            cost += self._F(x0=x_trj[i], p=u_trj[i])['qf'].__float__()

        cost += self._final_cost(x=x_trj[-1])['lf'].__float__()

        return cost

    def solve(self, niter):
        # type: (int) -> None

        if len(self._dcost) == 0:
            self._dcost.append(self._eval_cost(self._state_trj, self._ctrl_trj))

        for i in range(niter):

            self._linearize_quadratize()
            self._backward_pass()
            self._forward_pass()

            if self._verbose:
                print('Iter #{}: cost = {}'.format(i, self._dcost[-1]))

    def setInitialState(self, x0):
        # type: (np.array) -> None

        self._state_trj[0] = np.array(x0)

    def randomizeInitialGuess(self):

        self._state_trj[1:] = [np.random.randn(self._nx) for _ in range(self._N)]
        self._ctrl_trj  = [np.random.randn(self._nu) for _ in range(self._N)]

        if self._use_single_shooting_state_update:
            for i in range(self._N):
                self._state_trj[i+1] = self._F(x0=self._state_trj[i], p=self._ctrl_trj[i])['xf'].toarray().flatten()