try:
    from horizon.solvers.pyilqr import IterativeLQR
except ImportError as e:
    print(f'failed to import pyilqr extension: {e}')
    exit(1)

from horizon.variables import Parameter
from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.functions import Function, Constraint, ResidualFunction
from typing import Dict, List
from horizon.transcriptions import integrators
import casadi as cs
import numpy as np
from matplotlib import pyplot as plt

class SolverILQR(Solver):
    
    def __init__(self, 
                 prb: Problem,
                 opts: Dict = None) -> None:

        filtered_opts = None 
        if opts is not None:
            filtered_opts = {k: opts[k] for k in opts.keys() if k.startswith('ilqr.')}

        # init base class
        super().__init__(prb, filtered_opts)

        # save max iter if any
        self.max_iter = self.opts.get('ilqr.max_iter', 100)
        
        # num shooting interval
        self.N = prb.getNNodes() - 1  

        # get integrator and compute discrete dynamics in the form (x, u, p) -> f
        integrator_name = self.opts.get('ilqr.integrator', 'RK4')
        dae = {'ode': self.xdot, 'x': self.x, 'p': self.u, 'quad': 0}

        # handle parametric time
        integrator_opt = {}

        self.int = integrators.__dict__[integrator_name](dae, integrator_opt)
        if isinstance(self.dt, float):
            # integrator_opt['tf'] = self.dt
            x_int = self.int(self.x, self.u, self.dt)[0]
            dt_name = 'dt'
            time = cs.SX.sym(dt_name, 0)

        elif isinstance(self.dt, Parameter):
            time = cs.SX.sym(self.dt.getName(), 1)
            x_int = self.int(self.x, self.u, time)[0]
            dt_name = self.dt.getName()
            pass
        else:
            raise TypeError('ilqr supports only float and Parameter dt')

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
        self.dax = None
        self.hax = None

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

    def getDt(self):
        self.dt_solution = np.zeros(self.prb.getNNodes() - 1)
        if isinstance(self.dt, float):
            for node_n in range(self.prb.getNNodes() - 1):
                self.dt_solution[node_n] = self.dt
        elif isinstance(self.dt, Parameter):
            for node_n in range(self.prb.getNNodes() - 1):
                self.dt_solution[node_n] = self.dt.getValues(node_n)

        return self.dt_solution

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

        for fname, f in self.prb.function_container.getCost().items():
            self.ilqr.setIndices(fname, f.getNodes())

        for fname, f in self.prb.function_container.getCnstr().items():
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
            if isinstance(f, ResidualFunction):
                value = cs.sumsqr(f.getFunction()(*input_list, *param_list))
            else:
                value = f.getFunction()(*input_list, *param_list)

            # wrap function
            l = cs.Function(fname, 
                            [self.x, self.u] + param_list, [value], 
                            ['x', 'u'] + [p.getName() for p in param_list], 
                            [outname]
                            )


            set_to_ilqr(f.getNodes(), l)
        
    
    def _set_param_values(self):

        params = self.prb.var_container.getParList()
        for p in params:
            self.ilqr.setParameterValue(p.getName(), p.getValues())

    
    def _iter_callback(self, fpres):
        
        if not fpres.accepted:
            return

        fpres.print()

        if self.plot_iter:

            if self.xax is None:
                _, ((self.xax, self.uax), (self.dax, self.hax)) = plt.subplots(2, 2)
            
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

            plt.sca(self.dax)
            plt.cla()
            plt.plot(np.linalg.norm(fpres.defect_values, axis=1))
            plt.grid()
            plt.title(f'Dynamics gaps (iter {fpres.iter})')
            plt.xlabel('Node [-]')
            plt.ylabel('Gap')
            plt.legend([f'd{i}' for i in range(self.nx)])

            plt.sca(self.hax)
            plt.cla()
            plt.plot(fpres.constraint_values)
            plt.grid()
            plt.title(f'Constraint violation (iter {fpres.iter})')
            plt.xlabel('Node [-]')
            plt.ylabel('Constraint 1-norm')

            plt.draw()
            print("Press a button!")
            plt.waitforbuttonpress()

