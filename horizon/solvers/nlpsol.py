from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.variables import AbstractVariable, Variable, Parameter, SingleVariable, SingleParameter
from horizon.functions import CostFunction, ResidualFunction
from typing import Dict, List
import casadi as cs
import numpy as np
import pprint


class NlpsolSolver(Solver):
    
    def __init__(self, prb: Problem, opts: Dict, solver_plugin: str) -> None:
        
        super().__init__(prb, opts=opts)
        
        self.var_solution: Dict[str:np.array] = None
        self.cnstr_solution: Dict[str:np.array] = None
        self.dt_solution: np.array = None
        
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

        fun_sol_dict = dict()
        pos = 0
        for name, fun in self.fun_container.getCnstr().items():
            val_sol = sol['g'][pos:pos + fun.getDim() * len(fun.getNodes())]
            fun_sol_matrix = np.reshape(val_sol, (fun.getDim(), len(fun.getNodes())), order='F')
            fun_sol_dict[name] = fun_sol_matrix
            pos = pos + fun.getDim() * len(fun.getNodes())

        self.cnstr_solution = fun_sol_dict

        # retrieve state and input trajector
        input_vars = [v.getName() for v in self.prb.getInput().var_list]
        state_vars = [v.getName() for v in self.prb.getState().var_list]

        # get solution dict
        pos = 0
        var_sol_dict = dict()
        for var in self.var_container.getVarList(offset=False):
            val_sol = sol['x'][pos: pos + var.shape[0] * len(var.getNodes())]
            # this is to divide in rows the each dim of the var
            val_sol_matrix = np.reshape(val_sol, (var.shape[0], len(var.getNodes())), order='F')
            var_sol_dict[var.getName()] = val_sol_matrix
            pos = pos + var.shape[0] * len(var.getNodes())

        self.var_solution = var_sol_dict

        # get solution as state/input
        pos = 0
        for name, var in self.var_container.getVar().items():
            val_sol = sol['x'][pos: pos + var.shape[0] * len(var.getNodes())]
            val_sol_matrix = np.reshape(val_sol, (var.shape[0], len(var.getNodes())), order='F')
            if name in state_vars:
                off, _ = self.prb.getState().getVarIndex(name)
                self.x_opt[off:off+var.shape[0], :] = val_sol_matrix
            elif name in input_vars:
                off, _ = self.prb.getInput().getVarIndex(name)
                self.u_opt[off:off+var.shape[0], :] = val_sol_matrix
            else:
                pass
            pos = pos + var.shape[0] * len(var.getNodes())

        # build dt_solution as an array
        self.dt_solution = np.zeros(self.prb.getNNodes() - 1)
        dt = self.prb.getDt()

        # todo make this better
        # fill dt_solution

        # if it is a variable, its corresponding solution must be retrieved.
        # if it is a singleVariable or a singleParameter?
        # if dt is directly an optimization variable, that's ok, I get it from the var_solution
        # if dt is a function of some other optimization variables, get all of them and compute the optimized dt
        #   I do this by using a Function to wrap everything
        if isinstance(dt, cs.SX) and not isinstance(dt, Variable) and not isinstance(dt, SingleVariable):
            var_depend = list()
            for var in self.prb.getVariables().values():
                if cs.depends_on(dt, var):
                    var_depend.append(var)

            single_var_flag = False
            # check type of variable
            if all([isinstance(var, Variable) for var in var_depend]):
                pass
            elif all([isinstance(var, SingleVariable) for var in var_depend]):
                single_var_flag = True
            else:
                raise NotImplementedError('Yet to be done.')

            # create a function with all the variable dt depends on, and return dt
            temp_dt = cs.Function('temp_dt', var_depend, [dt])

            # fill the self.dt_solution with all the dt values
            node_n_out = 0
            for node_n in range(self.prb.getNNodes()-1):
                node_n_in = node_n

                # if the variable is a SingleVariable, fill dt_solution with the only value of the variable (node_n_out = 0)
                if not single_var_flag:
                    node_n_out = node_n

                self.dt_solution[node_n_in] = temp_dt(*[self.var_solution[var.getName()] for var in var_depend])[node_n_out]

        # fill the self.dt_solution with all the dt values
        elif isinstance(dt, Variable):
            for node_n in range(self.prb.getNNodes()-1):
                # from matrix to array
                sol_dt_array = self.var_solution[dt.getName()].flatten()
                self.dt_solution[node_n] = sol_dt_array[node_n]

        # fill the self.dt_solution with the same dt solution
        elif isinstance(dt, SingleVariable):
            for node_n in range(self.prb.getNNodes()-1):
                self.dt_solution[node_n] =  self.var_solution[dt.getName()]

        # if dt is a value, set it to each element of dt_solution
        elif isinstance(dt, Parameter) or isinstance(dt, SingleParameter):
            for node_n in range(self.prb.getNNodes()-1):
                # get only the nodes where the dt is selected
                # here dt at node 0 is not defined
                self.dt_solution[node_n] = dt.getValues(node_n)
        # if dt is a value, set it to each element of dt_solution
        elif isinstance(dt, (float, int)):
            for node_n in range(self.prb.getNNodes() - 1):
                self.dt_solution[node_n] = dt
        # if dt is a list, get each dt separately
        elif isinstance(dt, List):
            if len(dt) == self.prb.getNNodes() - 1:
                for node_n in range(self.prb.getNNodes()-1):
                    dt_val = dt[node_n]
                    if isinstance(dt_val, SingleVariable):
                        self.dt_solution[node_n] = self.var_solution[dt_val.getName()]
                    elif isinstance(dt_val, Variable):
                        current_node = dt_val.getNodes().index(node_n)
                        sol_var = self.var_solution[dt_val.getName()].flatten()[current_node]
                        self.dt_solution[node_n] = sol_var
                    else:
                        self.dt_solution[node_n] = dt[node_n]
            else:
                raise Exception(f'wrong dt list length: ({len(dt)}) != ({self.prb.getNNodes()-1})')
        else:
            raise ValueError(f'dt of type: {type(dt)} is not supported.')


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

    cost1 = prob.createCostFunction('cost1', x+p)
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


