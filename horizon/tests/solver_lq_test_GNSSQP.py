import unittest

from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.utils.transcription_methods import TranscriptionsHandler
import casadi as cs
import numpy as np

np.set_printoptions(suppress=True, precision=3)


class SolverConsistency(unittest.TestCase):
    def setUp(self) -> None:
        A11 = cs.DM.rand(2, 2)
        A13 = cs.DM.rand(2, 2)
        A21 = cs.DM.rand(2, 2)
        A32 = cs.DM.rand(2, 2)
        B21 = cs.DM.rand(2, 2)
        B32 = cs.DM.rand(2, 2)
        self.matrices = [A11, A13, A21, A32, B21, B32]

    # def _test_a_vs_b(self, a: Solver, b: Solver):
    #     a.solve()
    #     b.solve()
    #     xerr = a.x_opt - b.x_opt
    #     uerr = a.u_opt - b.u_opt
    #     self.assertLess(np.abs(xerr).max(), 1e-6)
    #     self.assertLess(np.abs(uerr).max(), 1e-6)
    #
    def test_gnsqp(self):
        gnsqp = make_problem('gnsqp', *self.matrices)
        gnsqp.solve()

    #
    # def test_blocksqp_vs_ilqr(self):
    #     ilqr = make_problem('ilqr', *self.matrices)
    #     blocksqp = make_problem('blocksqp', *self.matrices)
    #     self._test_a_vs_b(ilqr, blocksqp)


def make_problem(solver_type, A11, A13, A21, A32, B21, B32):
    # on a linear-quadratic problem, all solvers should agree on the solution
    N = 5
    dt = 0.1
    prob = Problem(N)

    # a random linear dynamics
    x1 = prob.createStateVariable('x1', dim=2)
    x2 = prob.createStateVariable('x2', dim=2)
    x3 = prob.createStateVariable('x3', dim=2)
    u1 = prob.createInputVariable('u1', dim=2)
    u2 = prob.createInputVariable('u2', dim=2)
    x = prob.getState().getVars()

    xdot = cs.vertcat(
        A11 @ x1 + A13 @ x3,
        A21 @ x1 + B21 @ u1,
        A32 @ x2 + B32 @ u2
    )
    prob.setDynamics(xdot)

    # a random cost
    prob.createIntermediateCost('c12', x1 + x2)
    prob.createIntermediateCost('c23', x2 + x3)
    prob.createIntermediateCost('c13', x1 + x3)
    prob.createIntermediateCost('u', cs.vertcat(u1 + u2))

    # a final constraint
    xtgt = np.array([1, 1, 2, 2, 3, 3])
    prob.createFinalConstraint('xtgt', x - xtgt)

    # an initial state
    x0 = -xtgt
    prob.getState().setBounds(lb=x0, ub=x0, nodes=0)


    # solver with sqp or ipopt need a dynamic constraint
    th = TranscriptionsHandler(prob, dt)
    th.setDefaultIntegrator(type='EULER')
    th.setMultipleShooting()

    # blocksqp needs exact hessian to be accurate
    opts = None

    bsqpsol = Solver.make_solver("gnsqp", prob, dt, opts)
    return bsqpsol


if __name__ == '__main__':
    unittest.main()