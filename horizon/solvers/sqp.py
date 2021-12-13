#try:
from .pysqp import SQPGaussNewtonSX
from horizon.solvers.pyilqr import IterativeLQR
#except ImportError:
#    print('failed to import pysqp extension; did you compile it?')
#    exit(1)

from .solver import Solver
from horizon.problem import Problem
from typing import Dict
import numpy as np
import casadi as cs


class GNSQPSolver(Solver):

    def __init__(self, prb: Problem, opts: Dict, qp_solver_plugin: str) -> None:

        super().__init__(prb, opts=opts)

        self.solution: Dict[str:np.array] = None
        self.prb = prb

        # generate problem to be solved
        self.var_container = self.prb.var_container
        self.fun_container = self.prb.function_container

        # generate problem to be solver
        var_list = list()
        for var in prb.var_container.getVarList(offset=False):
            var_list.append(var.getImpl())
        w = cs.vertcat(*var_list)  #

        fun_list = list()
        for fun in prb.function_container.getCnstr().values():
            fun_list.append(fun.getImpl())
        g = cs.vertcat(*fun_list)

        # build cost functions list
        cost_list = list()
        for fun in prb.function_container.getCost().values():
            cost_list.append(fun.getImpl())
        f = cs.vertcat(cs.vertcat(*cost_list))

        # create solver from prob
        F = cs.Function('f', [w], [f], ['x'], ['f'])
        G = cs.Function('g', [w], [g], ['x'], ['g'])
        self.solver = SQPGaussNewtonSX('gnsqp', qp_solver_plugin, F, G, self.opts)

    def set_iteration_callback(self, cb=None):
        if cb is None:
            self.solver.setIterationCallback(self._iter_callback)
        else:
            self.solver.setIterationCallback(cb)

    def _iter_callback(self, fpres):
            if not fpres.accepted:
                return
            fmt = ' <#09.3e'
            fmtf = ' <#04.2f'
            star = '*' if fpres.accepted else ' '
            fpres.print()

    def solve(self) -> bool:
        # update bounds and initial guess

        lb_list = list()
        for var in self.prb.var_container.getVarList(offset=False):
            lb_list.append(var.getLowerBounds().flatten(order='F'))
        lbw = cs.vertcat(*lb_list)

        # update upper bounds of variables
        ub_list = list()
        for var in self.prb.var_container.getVarList(offset=False):
            ub_list.append(var.getUpperBounds().flatten(order='F'))
        ubw = cs.vertcat(*ub_list)

        # update initial guess of variables
        w0_list = list()
        for var in self.prb.var_container.getVarList(offset=False):
            w0_list.append(var.getInitialGuess().flatten(order='F'))
        w0 = cs.vertcat(*w0_list)
        # to transform it to matrix form ---> vals = np.reshape(vals, (self.shape[0], len(self.nodes)), order='F')

        # update parameters
        p_list = list()
        for par in self.prb.var_container.getParList():
            p_list.append(par.getValues().flatten(order='F'))
        p = cs.vertcat(*p_list)

        # update lower bounds of constraints
        lbg_list = list()
        for fun in self.prb.function_container.getCnstr().values():
            lbg_list.append(fun.getLowerBounds().flatten(order='F'))
        lbg = cs.vertcat(*lbg_list)

        # update upper bounds of constraints
        ubg_list = list()
        for fun in self.prb.function_container.getCnstr().values():
            ubg_list.append(fun.getUpperBounds().flatten(order='F'))
        ubg = cs.vertcat(*ubg_list)

        # solve
        sol = self.solver.solve(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p)

        # retrieve state and input trajector
        input_vars = [v.getName() for v in self.prb.getInput().var_list]
        state_vars = [v.getName() for v in self.prb.getState().var_list]

        # get solution dict
        pos = 0
        solution_dict = dict()
        for var in self.var_container.getVarList(offset=False):
            val_sol = sol['x'][pos: pos + var.shape[0] * len(var.getNodes())]
            # this is to divide in rows the each dim of the var
            val_sol_matrix = np.reshape(val_sol, (var.shape[0], len(var.getNodes())), order='F')
            solution_dict[var.getName()] = val_sol_matrix
            pos = pos + var.shape[0] * len(var.getNodes())

        self.solution = solution_dict

        # get solution as state/input
        pos = 0
        for name, var in self.var_container.getVar().items():
            val_sol = sol['x'][pos: pos + var.shape[0] * len(var.getNodes())]
            val_sol_matrix = np.reshape(val_sol, (var.shape[0], len(var.getNodes())), order='F')
            if name in state_vars:
                off, _ = self.prb.getState().getVarIndex(name)
                self.x_opt[off:off + var.shape[0], :] = val_sol_matrix
            elif name in input_vars:
                off, _ = self.prb.getInput().getVarIndex(name)
                self.u_opt[off:off + var.shape[0], :] = val_sol_matrix
            else:
                pass
            pos = pos + var.shape[0] * len(var.getNodes())

        # print(f'{self.x_opt.shape}:, {self.x_opt}')
        # print(f'{self.u_opt.shape}:, {self.u_opt}')

        return True

    def getSolutionDict(self):
        return self.solution

    def getHessianComputationTime(self):
        return self.solver.getHessianComputationTime()

    def getQPComputationTime(self):
        return self.solver.getQPComputationTime()

    def getLineSearchComputationTime(self):
        return self.solver.getLineSearchComputationTime()

    def getObjectiveIterations(self):
        return self.solver.getObjectiveIterations()

    def getConstraintNormIterations(self):
        return self.solver.getConstraintNormIterations()

    def setAlphaMin(self, alpha_min):
        self.solver.setAlphaMin(alpha_min)

    def getAlpha(self):
        return self.solver.getAlpha()

    def getBeta(self):
        return self.solver.getBeta()

    def setBeta(self, beta):
        self.solver.setBeta(beta)
