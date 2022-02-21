from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.functions import CostFunction, ResidualFunction
from typing import Dict, List
import casadi as cs
import numpy as np
import pprint


class NlpsolSolver(Solver):
    
    def __init__(self, prb: Problem, opts: Dict, solver_plugin: str) -> None:
        
        super().__init__(prb, opts=opts)
        
        # generate problem to be solved
        self.var_container = self.prb.var_container
        self.fun_container = self.prb.function_container

        self.vars_impl = dict()
        self.pars_impl = dict()

        # dictionary of implemented variables

        j, w, g, p = self.build()
        # implement the abstract state variable with the current node
        # self.prb.var_container.build()
        # implement the constraints and the cost functions with the current node
        # self.function_container.build()

        # get j, w, g
        # j = self.function_container.getCostFImplSum()
        # w = self.var_container.getVarImplList()
        # g = self.function_container.getCnstrFList()
        # p = self.var_container.getParameterList()


        self.prob_dict = {'f': j, 'x': w, 'g': g, 'p': p}

        # create solver from prob
        self.solver = cs.nlpsol('solver', solver_plugin, self.prob_dict, self.opts)

    def build(self):
        """
        fill the dictionary "state_var_impl"
            - key: nodes (nNone, n0, n1, ...) nNone contains single variables that are not projected in nodes
            - val: dict with name and value of implemented variable
        """

        # todo it seems tht i only need self.vars in var_container.
        # ORDERED AS VARIABLES
        # build variables
        var_list = list()
        for var in self.var_container.getVarList(offset=False):
            var_list.append(var.getImpl())
        w = cs.vertcat(*var_list)


        # build parameters
        par_list = list()
        for par in self.var_container.getParList(offset=False):
            par_list.append(par.getImpl())
        p = cs.vertcat(*par_list)

        # build constraint functions list
        fun_list = list()
        for fun in self.fun_container.getCnstr().values():
            fun_list.append(fun.getImpl())
        g = cs.vertcat(*fun_list)

        # treat differently cost and residual (residual must be quadratized)
        fun_list = list()
        for fun in self.fun_container.getCost().values():
            if isinstance(fun, CostFunction):
                fun_list.append(fun.getImpl())
            elif isinstance(fun, ResidualFunction):
                fun_list.append(cs.sumsqr(fun.getImpl()))
            else:
                raise Exception('wrong type of function found in fun_container')

        j = cs.sum1(cs.vertcat(*fun_list))

        return j, w, g, p


    def solve(self) -> bool:

        # update lower/upper bounds of variables
        lbw = self._getVarList('lb')
        ubw = self._getVarList('ub')
        # update initial guess of variables
        w0 = self._getVarList('ig')
        # update parameters
        p = self._getParList()
        # update lower/upper bounds of constraints
        lbg = self._getFunList('lb')
        ubg = self._getFunList('ub')

        # solve
        sol = self.solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p)

        self.cnstr_solution = self._createCnsrtSolDict(sol)

        # retrieve state and input trajector


        # get solution dict
        self.var_solution = self._createVarSolDict(sol)


        # get solution as state/input
        self._createVarSolAsInOut(sol)

        # build dt_solution as an array
        self._createDtSol()

        return True

    def getSolutionDict(self):
        return self.var_solution

    def getConstraintSolutionDict(self):
        return self.cnstr_solution

    def getDt(self):
        return self.dt_solution

if __name__ == '__main__':

    # from matplotlib import pyplot as plt
    #
    # # create problem
    # N = 100
    # dt = 0.03
    # prb = Problem(N)
    #
    # # create variables
    # p = prb.createStateVariable('p', 2)
    # theta = prb.createStateVariable('theta', 1)
    # v = prb.createInputVariable('v', 1)
    # omega = prb.createInputVariable('omega', 1)
    #
    # p.setBounds([99, 99], [99, 99], nodes=50)
    # # define dynamics
    # x = prb.getState().getVars()
    # u = prb.getInput().getVars()
    # xdot = cs.vertcat(v * cs.cos(theta),
    #                   v * cs.sin(theta),
    #                   omega)
    # prb.setDynamics(xdot)
    #
    # # Cost function
    # x_tgt = np.array([1, 0, 0])
    # prb.createIntermediateCost("reg", 1e-6 * cs.sumsqr(u))
    # prb.createFinalConstraint("gothere", x - x_tgt)
    #
    # # initial state
    # x0 = np.array([0, 0, np.pi / 2])
    # prb.setInitialState(x0=x0)
    #
    # # TEST ILQR
    # sol = NlpsolSolver(prb, dt, {}, 'ipopt')
    # sol.solve()
    # print(sol.x_opt.shape)
    # print(sol.x_opt)
    # # print(sol.u_opt)

    # exit()

    N = 10
    dt = 0.01
    prob = Problem(10)
    x = prob.createStateVariable('x', 2)
    y = prob.createStateVariable('y', 4)
    u = prob.createInputVariable('u', 2)
    z = prob.createSingleVariable('z', 4)
    j = prob.createSingleParameter('j', 1)
    p = prob.createParameter('p', 2)

    z.setBounds([77, 77, 77, 77], [77, 77, 77, 77])
    x_next = x.getVarOffset(1)
    x_prev = x.getVarOffset(-1)
    f = prob.createSingleParameter('f', 4)
    #
    a = prob.createVariable('a', 2, nodes=range(0, 5))

    x.setInitialGuess([1, 1], nodes=0)
    x.setInitialGuess([10, 10], nodes=10)
    a.setBounds([0, 0], [5, 5])

    p.assign([7, 7], nodes=range(0, 4))
    p.assign([2, 2], nodes=4)

    j.assign([44])

    print(z.getUpperBounds(range(3, 5)))
    #
    cnsrt0 = prob.createIntermediateConstraint('cnsrt0', y[2:4] + u)
    ## =========
    cnsrt1 = prob.createIntermediateConstraint('cnsrt1', x + u)
    cnsrt1.setLowerBounds([-np.inf, -np.inf])
    ## this is new, bitches!
    print(cnsrt1.getImpl(2)) # the constraints get implemented as soon as it get created muahahah
    ## =========
    # cnsrt2 = prob.createConstraint('cnsrt2', x * y[0:2], nodes=[3, 8])
    ## =========
    # cnsrt3 = prob.createConstraint('cnsrt3', x + p)
    ## =========
    # cnsrt4 = prob.createConstraint('cnsrt4', x + f[0:2])
    ## =========
    # cnsrt5 = prob.createConstraint('cnsrt5', p + f[0:2] + z[2:4])
    ## =========
    # cnsrt6 = prob.createConstraint('cnsrt6', x + z[0:2])
    ## =========
    # this should be the same
    # cnsrt7 = prob.createIntermediateConstraint('cnsrt7', x_next - x)
    # cnsrt8 = prob.createConstraint('cnsrt8', x - x_prev, nodes=range(1, N+1))
    # cnsrt9 = prob.createConstraint('cnsrt9', y, nodes=N)
    #

    cost1 = prob.createCost('cost1', x+p)
    # =========

    # todo check if everything is allright!
    for i in range(N):
        x.setLowerBounds(np.array(range(i, i+2)), nodes=i)

    p.assign([20, 20], nodes=4)
    # f.assign([121, 122, 120, 119])
    xdot = cs.vertcat(y, u)
    # xdot = cs.vertcat(u)
    prob.setDynamics(xdot)
    sol = NlpsolSolver(prb=prob, opts=dict(), solver_plugin='ipopt')
    sol.solve()


