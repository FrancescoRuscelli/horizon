from horizon.problem import Problem
from horizon.solvers import Solver
from ..transcriptions import integrators
import numpy as np

class RealTimeIteration:

    def __init__(self, prb: Problem, solver: Solver, dt: float) -> None:
        # define integrator to simulate the system (should be optional)
        x = prb.getState().getVars()
        u = prb.getInput().getVars()
        self.dt = dt
        xdot = prb.getDynamics()
        dae = {'x': x, 'p': u, 'ode': xdot, 'quad': 0}
        self.integrator = integrators.RK4(dae)
        
        self.solution = None
        self.state = prb.getState()
        self.prb = prb
        self.solver = solver

    def run(self, stateread: np.array):
    
        # set initial state
        self.state.setBounds(lb=stateread, ub=stateread, nodes=0)
        
        # solve
        self.solver.solve()

        # get control input to apply
        u_opt = self.solver.u_opt[:, 0]

        # return
        return u_opt

    def integrate(self, stateread: np.array, u_opt: np.array):
        # integrate input and return
        stateint = self.integrator(x0=stateread, p=u_opt, time=self.dt)['xf'].toarray().flatten()
        return stateint