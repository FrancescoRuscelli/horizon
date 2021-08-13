try:
    from horizon.solvers.pyilqr import IterativeLQR
except ImportError:
    print('failed to import pyilqr extension; did you compile it?')
    exit(1)

from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.functions import Function, CostFunction, Constraint
from typing import Dict, List
from horizon.utils import integrators
import casadi as cs
import numpy as np

class SolverILQR(Solver):
    
    def __init__(self, 
                 prb: Problem, 
                 dt: float, 
                 opts: Dict = None) -> None:

        # init base class
        super().__init__(prb, dt, opts)

        # save max iter if any
        self.max_iter = self.opts.get('max_iter', 10)
        
        # num shooting interval
        self.N = prb.getNNodes() - 1  

        # get integrator and compute discrete dynamics in the form (x, u) -> f
        integrator_name = self.opts.get('ilqr.integrator', 'EULER')
        dae = {'ode': self.xdot, 'x': self.x, 'p': self.u, 'quad': 0}
        self.int = integrators.__dict__[integrator_name](dae, {'tf': dt})
        self.dyn = cs.Function('f', {'x': self.x, 'u': self.u, 'f': self.int(self.x, self.u)[0]},
                               ['x', 'u'], ['f'])

        # create ilqr solver
        self.ilqr = IterativeLQR(self.dyn, self.N)

        # set costs and constraints
        for k in range(self.N + 1):
            self._set_cost_k(k)
            self._set_constraint_k(k)

    def configure_rti(self) -> bool:
        self.opts['max_iter'] = 1
    
    def solve(self):
        x0 = self.prb.getInitialState().reshape((self.nx, 1))
        self.x_opt = np.hstack(([x0]*(self.N+1)))
        self.ilqr.setStateInitialGuess(self.x_opt)
        self.ilqr.setIterationCallback(self._iter_callback)
        self.ilqr.solve(self.max_iter)
        self.x_opt = self.ilqr.getStateTrajectory()
        self.u_opt = self.ilqr.getInputTrajectory()
        

    def print_timings(self):

        prof_info = self.ilqr.getProfilingInfo()

        if len(prof_info.timings) == 0:
            return
        
        print('\ntimings (inner):')
        for k, v in prof_info.timings.items():
            if '_inner' not in k:
                continue
            print(f'{k[:-6]:30}{np.mean(v)} us')

        print('\ntimings (iter):')
        for k, v in prof_info.timings.items():
            if '_inner' in k:
                continue
            print(f'{k:30}{np.mean(v)} us')
    
    
    def _set_cost_k(self, k):
        
        self._set_fun_k(k, 
                container=self.prb.function_container.costfun_container, 
                set_to_ilqr=self.ilqr.setIntermediateCost, 
                combine_elements=sum,
                outname='l')

    
    def _set_constraint_k(self, k):
        
        def vertcat_list(l: List):
            return cs.vertcat(*l)

        self._set_fun_k(k, 
                container=self.prb.function_container.cnstr_container, 
                set_to_ilqr=self.ilqr.setIntermediateConstraint,
                combine_elements=vertcat_list,
                outname='h')
    
    
    def _set_fun_k(self, k, container, set_to_ilqr, combine_elements, outname):

        value_list = list()

        # check state and input bounds
        if outname == 'h':

            # state
            xlb, xub = self.prb.getState().getBounds(node=k)
            if np.all(xlb == xub):
                value_list.append(self.x - xlb)

            # input
            if k < self.N:
                ulb, uub = self.prb.getInput().getBounds(node=k)
                if np.all(ulb == uub):
                    value_list.append(self.u - ulb)

        
        # check constraints    
        for fname, f in container.items():
            
            # give a type to f
            f: Function = f

            # if f does not act on node k, skip
            if k not in f.nodes:
                continue
                
            # get input variables for this function
            input_list = f.getVariables()

            # save function value to list
            value = f.fun(*input_list)

            # in the case of constraints, check bound values
            if isinstance(f, Constraint):
                lb, ub = Constraint.getBounds(f, nodes=k)
                if np.any(lb != ub):
                    raise ValueError(f'[ilqr] constraint {fname} not an equality constraint')
                value -= lb

            value_list.append(value)
        
        # if empty, skip
        if len(value_list) == 0:
            return 
        
        # compute overall value
        total_value = combine_elements(value_list)
    
        # wrap function
        l = cs.Function(f'{outname}_{k}', [self.x, self.u], [total_value], 
                            ['x', 'u'], [outname])

        # set it to solver
        set_to_ilqr(k, l)


    
    def _iter_callback(self, xtrj, utrj, du, cost, defect, constr):
        fmt = ' <#010.3f'
        print(f'delta_u={du:{fmt}}  cost={cost:{fmt}}  constr={constr:{fmt}}  gap={defect:{fmt}}')


                    
                
                


            

############# TESTING STUFF TO BE REMOVED #######################
if __name__ == '__main__':

    from matplotlib import pyplot as plt

    # create problem
    N = 100
    dt = 0.03
    prb = Problem(N)

    # create variables
    p = prb.createStateVariable('p', 2)
    theta = prb.createStateVariable('theta', 1)
    v = prb.createInputVariable('v', 1)
    omega = prb.createInputVariable('omega', 1)

    # define dynamics 
    x = prb.getState().getVars()
    u = prb.getInput().getVars()
    xdot = cs.vertcat(v*cs.cos(theta), 
                    v*cs.sin(theta),
                    omega)
    prb.setDynamics(xdot)

    # Cost function
    x_tgt = np.array([1, 0, 0])
    prb.createIntermediateCost("reg", 1e-6*cs.sumsqr(u))
    prb.createFinalConstraint("gothere", x - x_tgt)

    # initial state
    x0 = np.array([0, 0, np.pi/2])
    prb.setInitialState(x0=x0)

    # TEST ILQR
    sol = SolverILQR(prb, dt)
    sol.solve()
    sol.print_timings()

    plt.plot(sol.ilqr.getStateTrajectory().T, '-')
    plt.show()