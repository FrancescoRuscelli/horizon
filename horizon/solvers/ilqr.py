try:
    from horizon.solvers.pyilqr import IterativeLQR
except ImportError as e:
    print(f'failed to import pyilqr extension: {e}')
    exit(1)

from horizon.variables import Parameter
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

        filtered_opts = None 
        if opts is not None:
            filtered_opts = {k: opts[k] for k in opts.keys() if k.startswith('ilqr.')}

        # init base class
        super().__init__(prb, dt, filtered_opts)

        # save max iter if any
        self.max_iter = self.opts.get('ilqr.max_iter', 100)
        
        # num shooting interval
        self.N = prb.getNNodes() - 1  

        # get integrator and compute discrete dynamics in the form (x, u, p) -> f
        integrator_name = self.opts.get('ilqr.integrator', 'RK4')
        dae = {'ode': self.xdot, 'x': self.x, 'p': self.u, 'quad': 0}

        # handle parametric time
        integrator_opt = {}
        if isinstance(dt, float):
            integrator_opt['tf'] = dt 
        elif isinstance(dt, Parameter):
            pass
        else:
            raise TypeError('ilqr supports only float and Parameter dt')

        self.int = integrators.__dict__[integrator_name](dae, integrator_opt)

        # handle possible parametric time 
        dt_name = 'dt'
        if self.int.n_in() == 3:
            time = cs.SX.sym(dt.getName(), 1)
            dt_name = dt.getName()
            x_int = self.int(self.x, self.u, time)[0]
        elif self.int.n_in() == 2:
            time = cs.SX.sym(dt_name, 0)
            x_int = self.int(self.x, self.u)[0]
        else:
            raise IndexError('integrated dynamics should either have 2 or 3 inputs')


        self.dyn = cs.Function('f', 
                               {'x': self.x, 'u': self.u, dt_name: time, 'f': x_int},
                               ['x', 'u', dt_name], ['f']
                               )

        # create ilqr solver
        self.ilqr = IterativeLQR(self.dyn, self.N, self.opts)

        # set constraints, costs, bounds
        self._set_constraint()
        self._set_cost()
        self._set_bounds()
        

        # set a default iteration callback
        self.plot_iter = False
        self.xax = None 
        self.uax = None

        # empty solution dict
        self.solution_dict = dict()

    def save(self):
        data = self.prb.save()
        data['solver'] = dict()
        if isinstance(self.dt, float):
            data['solver']['dt'] = self.dt
        data['solver']['name'] = 'ilqr'
        data['solver']['opt'] = self.opts
        data['dynamics'] = self.dyn.serialize()
        return data

    
    def set_iteration_callback(self, cb=None):
        if cb is None:
            self.ilqr.setIterationCallback(self._iter_callback)
        else:
            self.ilqr.setIterationCallback(cb)


    def configure_rti(self) -> bool:
        self.opts['max_iter'] = 1
    
    def solve(self):
        
        # set initial state
        x0 = self.prb.getInitialState()
        xinit = self.prb.getState().getInitialGuess()
        uinit = self.prb.getInput().getInitialGuess()
        xinit[:, 0] = x0.flatten()

        # update initial guess
        self.ilqr.setStateInitialGuess(xinit)
        self.ilqr.setInputInitialGuess(uinit)
        self.ilqr.setIterationCallback(self._iter_callback)
        
        # update parameters
        self._set_param_values()

        # update bounds
        self._set_bounds()

        # update nodes
        self._update_nodes()

        # solve
        ret = self.ilqr.solve(self.max_iter)

        # get solution
        self.x_opt = self.ilqr.getStateTrajectory()
        self.u_opt = self.ilqr.getInputTrajectory()

        # populate solution dict
        for var in self.prb.getState().var_list:
            vname = var.getName()
            off, dim = self.prb.getState().getVarIndex(vname)
            self.solution_dict[vname] = self.x_opt[off:off+dim, :]
            
        for var in self.prb.getInput().var_list:
            vname = var.getName()
            off, dim = self.prb.getInput().getVarIndex(vname)
            self.solution_dict[vname] = self.u_opt[off:off+dim, :]

        return ret
    
    def getSolutionDict(self):
        return self.solution_dict

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
            print(f'{k:30}{np.mean(v)*1e-3} ms')

    def _update_nodes(self):

        print('updating nodes..')

        for fname, f in self.prb.function_container.getCost().items():
            print(f'{fname}: {f.getNodes()}')
            self.ilqr.setIndices(fname, f.getNodes())

        for fname, f in self.prb.function_container.getCnstr().items():
            print(f'{fname}: {f.getNodes()}')
            self.ilqr.setIndices(fname, f.getNodes())

        self.ilqr.updateIndices()
    
    
    def _set_cost(self):
        
        self._set_fun(container=self.prb.function_container.getCost(),
                set_to_ilqr=self.ilqr.setIntermediateCost, 
                outname='l')

    
    def _set_constraint(self):

        self._set_fun(container=self.prb.function_container.getCnstr(),
                set_to_ilqr=self.ilqr.setIntermediateConstraint,
                outname='h')

    def _set_bounds(self):

        xlb, xub = self.prb.getState().getBounds(node=None)
        ulb, uub = self.prb.getInput().getBounds(node=None)
        self.ilqr.setStateBounds(xlb, xub)
        self.ilqr.setInputBounds(ulb, uub)

    def _set_fun(self, container, set_to_ilqr, outname):

        # check fn in container    
        for fname, f in container.items():
            
            # give a type to f
            f: Function = f

            # get input variables for this function
            input_list = f.getVariables()
            param_list = f.getParameters()

            # save function value
            value = f.getFunction()(*input_list, *param_list)

            # wrap function
            l = cs.Function(fname, 
                            [self.x, self.u] + param_list, [value], 
                            ['x', 'u'] + [p.getName() for p in param_list], 
                            [outname]
                            )


            set_to_ilqr([], l)
        
    
    def _set_param_values(self):

        params = self.prb.var_container.getParList()
        for p in params:
            print(p.getName(), p.getValues())
            self.ilqr.setParameterValue(p.getName(), p.getValues())

    
    def _iter_callback(self, fpres):
        if not fpres.accepted:
            return
        fpres.print()

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
            print("Press a button!")
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
