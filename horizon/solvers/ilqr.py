try:
    from horizon.solvers.pyilqr import IterativeLQR
except ImportError:
    print('failed to import pyilqr extension; did you compile it?')
    exit(1)

from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.functions import Function, Constraint
from typing import Dict, List
from horizon.transcriptions import integrators
import casadi as cs
import numpy as np
from matplotlib import pyplot as plt

class SolverILQR(Solver):
    
    def __init__(self, 
                 prb: Problem, 
                 dt: float, 
                 opts: Dict = None) -> None:

        # init base class
        super().__init__(prb, dt, opts)

        # save max iter if any
        self.max_iter = self.opts.get('max_iter', 100)
        
        # num shooting interval
        self.N = prb.getNNodes() - 1  

        # get integrator and compute discrete dynamics in the form (x, u) -> f
        integrator_name = self.opts.get('ilqr.integrator', 'RK4')
        dae = {'ode': self.xdot, 'x': self.x, 'p': self.u, 'quad': 0}
        self.int = integrators.__dict__[integrator_name](dae, {'tf': dt})
        self.dyn = cs.Function('f', {'x': self.x, 'u': self.u, 'f': self.int(self.x, self.u)[0]},
                               ['x', 'u'], ['f'])

        # create ilqr solver
        self.ilqr = IterativeLQR(self.dyn, self.N, self.opts)

        # set costs and constraints
        for k in range(self.N + 1):
            self._set_cost_k(k)
            self._set_constraint_k(k)

        # set a default iteration callback
        self.plot_iter = False
        self.xax = None 
        self.uax = None

    
    def set_iteration_callback(self, cb=None):
        if cb is None:
            self.ilqr.setIterationCallback(self._iter_callback)
        else:
            self.ilqr.setIterationCallback(cb)


    def configure_rti(self) -> bool:
        self.opts['max_iter'] = 1
    
    def solve(self):
        x0 = self.prb.getInitialState().reshape((self.nx, 1))
        self.x_opt = np.hstack(([x0]*(self.N+1)))
        self.ilqr.setStateInitialGuess(self.x_opt)
        self.ilqr.setIterationCallback(self._iter_callback)
        ret = self.ilqr.solve(self.max_iter)

        # get solution
        self.x_opt = self.ilqr.getStateTrajectory()
        self.u_opt = self.ilqr.getInputTrajectory()

        return ret
        

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
                container=self.prb.function_container.getCost(),
                set_to_ilqr=self.ilqr.setIntermediateCost, 
                outname='l')

    
    def _set_constraint_k(self, k):

        self._set_fun_k(k, 
                container=self.prb.function_container.getCnstr(),
                set_to_ilqr=self.ilqr.setIntermediateConstraint,
                outname='h')
    
    
    def _set_fun_k(self, k, container, set_to_ilqr, outname):

        # check state and input bounds
        if outname == 'h':

            # state
            xlb, xub = self.prb.getState().getBounds(node=k)
            if np.all(xlb == xub) and k > 0:  # note: skip initial condition
                l = cs.Function(f'xc_{k}', [self.x, self.u], [self.x - xlb], 
                                ['x', 'u'], [outname])
                set_to_ilqr(k, l)

            # input
            if k < self.N:
                ulb, uub = self.prb.getInput().getBounds(node=k)
                if np.all(ulb == uub):
                    l = cs.Function(f'uc_{k}', [self.x, self.u], [self.u - ulb], 
                                ['x', 'u'], [outname])
                    set_to_ilqr(k, l)

        
        # check fn in container    
        for fname, f in container.items():
            
            # give a type to f
            f: Function = f

            # if f does not act on node k, skip
            if k not in f.getNodes():
                continue
                
            # get input variables for this function
            input_list = f.getVariables()

            # save function value to list
            value = f.getFunction()(*input_list)

            # in the case of constraints, check bound values
            if isinstance(f, Constraint):
                lb, ub = Constraint.getBounds(f, nodes=k)
                if np.any(lb != ub):
                    raise ValueError(f'[ilqr] constraint {fname} not an equality constraint')
                value -= lb

            # wrap function
            l = cs.Function(f'{fname}_{k}', [self.x, self.u], [value], 
                                ['x', 'u'], [outname])

            # set it to solver
            set_to_ilqr(k, l)
        

    
    def _iter_callback(self, fpres):
        # if not fpres.accepted:
        #     return
        fmt = ' <#010.3e'
        fmtf = ' <#06.3f'
        star = '*' if fpres.accepted else ' '
        print(f'{star}\
alpha={fpres.alpha:{fmtf}}  \
reg={fpres.hxx_reg:{fmt}}  \
merit={fpres.merit:{fmt}}  \
dm={fpres.merit_der:{fmt}}  \
mu_f={fpres.mu_f:{fmt}}  \
mu_c={fpres.mu_c:{fmt}}  \
cost={fpres.cost:{fmt}}  \
delta_u={fpres.step_length:{fmt}}  \
constr={fpres.constraint_violation:{fmt}}  \
gap={fpres.defect_norm:{fmt}}')

        if self.plot_iter and fpres.accepted:

            if self.xax is None:
                _, (self.xax, self.uax) = plt.subplots(1, 2)
            
            plt.sca(self.xax)
            plt.cla()
            plt.plot(fpres.xtrj.T)
            plt.grid()
            plt.title(f'State trajectory (iter {fpres.iter})')
            plt.xlabel('Node [-]')
            plt.ylabel('State')
            plt.legend([f'x{i}' for i in range(self.nx)])

            plt.sca(self.uax)
            plt.cla()
            plt.plot(fpres.utrj.T)
            plt.grid()
            plt.title(f'Input trajectory (iter {fpres.iter})')
            plt.xlabel('Node [-]')
            plt.ylabel('Input')
            plt.legend([f'u{i}' for i in range(self.nu)])
            plt.draw()
            plt.waitforbuttonpress()


                    
                
                


            

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
