import unittest

from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.transcriptions.transcriptor import Transcriptor
import casadi as cs
import numpy as np
np.set_printoptions(suppress=True, precision=3)

class EqualityConstrained(unittest.TestCase):
    def setUp(self) -> None:
        
        # particle subject to a force about +x axis,
        # and constrained to a x2 = x1^2 parabola
        # m*ddx + m*g = [tau, 0] + Jc'*f
        
        N = 10
        dt = 0.1
        prb = Problem(N)
        x = prb.createStateVariable('x', 2)
        v = prb.createStateVariable('v', 2)
        a = prb.createInputVariable('a', 2)
        f = prb.createInputVariable('f', 1)
        xdot = cs.vertcat(v, a) 
        prb.setDynamics(xdot)

        x0 = np.array([0, 0, 0, 0])
        prb.setInitialState(x0)

        contact = x[1] - x[0]**2
        Jc = cs.horzcat(-2*x[0], 1)
        prb.createIntermediateConstraint('contact', contact, nodes=range(1, N+1))

        underactuation = a[1] + 1.0 - Jc[1]*f
        prb.createIntermediateConstraint('underactuation', underactuation)

        prb.createIntermediateCost('min_acc', cs.sumsqr(a))

        goal = 2.0
        prb.createFinalConstraint('goal', x[0] - goal)
        prb.createFinalConstraint('vf', v)

        solver = Solver.make_solver('ilqr', 
            prb, 
            dt, 
            opts={
                'ilqr.integrator': 'RK4'
            })

        self.solver = solver
        self.prb = prb

        

    def test_simple_constrained(self):
        # self.solver.plot_iter = True
        self.solver.set_iteration_callback()
        ret = self.solver.solve()
        self.assertTrue(ret)
        
    
  


if __name__ == '__main__':
    unittest.main()