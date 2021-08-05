from pyilqr import IterativeLQR
from horizon.problem import Problem
from horizon.function import Function
from typing import Dict
from horizon.utils import integrators
import casadi as cs
import numpy as np

class SolverILQR:
    
    def __init__(self, 
                 prb: Problem, 
                 dt: float, 
                 opts: Dict = None) -> None:

        # handle inputs
        if opts is None:
            opts = dict()
        
        self.prb = prb
        
        # num shooting interval
        self.N = prb.getNNodes() - 1  

        # get state and control
        self.x = prb.getState().getVars()
        self.u = prb.getInput().getVars()
        self.xdot = prb.getDynamics()

        # get integrator and compute discrete dynamics in the form (x, u) -> f
        integrator_name = opts.get('ilqr.integrator', 'EULER')
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

    
    def solve(self):
        self.ilqr.setIterationCallback(self._iter_callback)
        self.ilqr.solve(5)

    def print_timings(self):

        prof_info = self.ilqr.getProfilingInfo()
        
        print('\n\ntimings (inner):')
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
                self.prb.function_container.costfun_container, 
                self.ilqr.setIntermediateCost, 
                'l')

    
    def _set_constraint_k(self, k):
        self._set_fun_k(k, 
                self.prb.function_container.cnstr_container, 
                self.ilqr.setIntermediateConstraint,
                'h')
    
    
    def _set_fun_k(self, k, container, set_to_ilqr, outname):
                
        for fname, f in container.items():
            
            # give a type to f
            f: Function = f

            # if f does not act on node k, skip
            if k not in f.nodes:
                continue
                
            # get input variables for this function
            input_vars = f.getVariables()

            # make input list dor this function
            input_list = list()
            for vname, vlist in input_vars.items():
                var: cs.SX = vlist[0]
                input_list.append(var)

            # wrap function
            l = cs.Function(fname, [self.x, self.u], [f.fun(*input_list)[0]], 
                            ['x', 'u'], [outname])

            # set it to solver
            set_to_ilqr(k, l)

    
    def _iter_callback(self, xtrj, utrj, du, cost, defect, constr):
        fmt = ' <#010.3f'
        print(f'delta_u={du:{fmt}}  cost={cost:{fmt}}  constr={constr:{fmt}}  gap={defect:{fmt}}')


                    
                
                


            



############# TESTING STUFF TO BE REMOVED #######################

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
x_tgt = np.array([1.0, 0.0, 0.0])
prb.createIntermediateCost("reg", 1e-6*cs.sumsqr(u))
prb.createFinalConstraint("gothere", x - x_tgt)

# TEST ILQR
sol = SolverILQR(prb, dt)
x0 = np.array([0, 0, np.pi/2])
sol.ilqr.setInitialState(x0)
sol.solve()
sol.print_timings()