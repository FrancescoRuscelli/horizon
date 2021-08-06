import time

import casadi as cs
from horizon import function as fc
from horizon import nodes as nd
from horizon import state_variables as sv
import numpy as np
import logging
import sys
import pickle
import horizon.misc_function as misc

class Problem:

    def __init__(self, N, crash_if_suboptimal=False, logging_level=logging.INFO):

        self.opts = None
        self.solver = None
        self.__solution = None
        self.sol = None # store solution from solver
        self.default_solver = cs.nlpsol
        self.default_solver_plugin = 'ipopt'

        self.logger = logging.getLogger('logger')
        self.logger.setLevel(level=logging_level)
        self.debug_mode = self.logger.isEnabledFor(logging.DEBUG)
        stdout_handler = logging.StreamHandler(sys.stdout)
        self.logger.addHandler(stdout_handler)

        self.crash_if_suboptimal = crash_if_suboptimal

        self.nodes = N + 1
        # state variable to optimize
        self.var_container = sv.VariablesContainer(self.nodes)
        self.function_container = fc.FunctionsContainer(self.var_container, self.nodes, self.logger)
        self.prob = None

        self.state_aggr = sv.StateAggregate()
        self.input_aggr = sv.InputAggregate()
        self.state_der: cs.SX = None

    def createStateVariable(self, name, dim):
        if self.state_der is not None:
            raise RuntimeError('createStateVariable must be called *before* setDynamics')
        var = self.var_container.setStateVar(name, dim)
        self.state_aggr.addVariable(var)
        return var

    def createInputVariable(self, name, dim):
        var = self.var_container.setInputVar(name, dim)
        self.input_aggr.addVariable(var)
        return var

    def createSingleVariable(self, name, dim):
        var = self.var_container.setSingleVar(name, dim)
        return var

    def createVariable(self, name, dim, nodes=None):
        var = self.var_container.setVar(name, dim, nodes)
        return var

    def createParameter(self, name, dim, nodes=None):
        par = self.var_container.setParameter(name, dim, nodes)
        return par

    def createSingleParameter(self, name, dim):
        par = self.var_container.setSingleParameter(name, dim)
        return par
    # def setVariable(self, name, var):

    # assert (isinstance(var, (cs.casadi.SX, cs.casadi.MX)))
    # setattr(Problem, name, var)
    # self.var_container.append(name)

    # def getStateVariable(self, name):
    #
    #     for var in self.var_container:
    #         if var.getName() == name:
    #             return var
    #     return None
    def getState(self) -> sv.StateAggregate:
        return self.state_aggr

    def getInput(self) -> sv.InputAggregate:
        return self.input_aggr

    def setDynamics(self, xdot: cs.SX):
        nx = self.getState().getVars().shape[0]
        if xdot.shape[0] != nx:
            raise ValueError(f'state derivative dimension mismatch ({xdot.shape[0]} != {nx})')
        self.state_der = xdot

    def getDynamics(self) -> cs.SX:
        return self.state_der

    def _getUsedVar(self, f):
        used_var = dict()
        for name_var, value_var in self.var_container.getVarAbstrDict().items():
            for var in value_var:
                if cs.depends_on(f, var):
                    if name_var not in used_var:
                        used_var[name_var] = list()
                    used_var[name_var].append(var)

        return used_var

    def _getUsedPar(self, f):
        used_par = dict()
        for name_par, value_par in self.var_container.getParAbstrDict().items():
            if cs.depends_on(f, value_par):
                if name_par not in used_par:
                    used_par[name_par] = list()
                used_par[name_par].append(value_par)

        return used_par

    # @classmethod
    # def createFunction(self, fun_type, **kwargs):
    #         return self.function[fun_type](**kwargs)

    def createConstraint(self, name, g, nodes=None, bounds=None):

        if nodes is None:
            nodes = range(self.nodes)
        else:
            nodes = misc.checkNodes(nodes, range(self.nodes))

        # get vars that constraint depends upon
        used_var = self._getUsedVar(g)
        used_par = self._getUsedPar(g)

        if self.debug_mode:
            self.logger.debug(f'Creating Constraint Function "{name}": active in nodes: {nodes} using vars {used_var}')

        # create internal representation of a constraint
        fun = fc.Constraint(name, g, used_var, used_par, nodes, bounds)

        self.function_container.addFunction(fun)

        return fun

    def createFinalConstraint(self, name, g, bounds=None):
        u = self.getInput().getVars()
        if cs.depends_on(g, u):
            raise RuntimeError(f'final constraint "{name}" must not depend on the input')
        return self.createConstraint(name, g, nodes=self.nodes-1, bounds=bounds)

    def createIntermediateConstraint(self, name, g, nodes=None, bounds=None):
        if nodes is None:
            nodes = range(self.nodes-1)
        return self.createConstraint(name, g, nodes=nodes, bounds=bounds)    

    def createCostFunction(self, name, j, nodes=None):

        if nodes is None:
            nodes = range(self.nodes)
        else:
            nodes = misc.checkNodes(nodes, range(self.nodes))

        used_var = self._getUsedVar(j)
        used_par = self._getUsedPar(j)

        if self.debug_mode:
            self.logger.debug(f'Creating Cost Function "{name}": active in nodes: {nodes}')

        fun = fc.CostFunction(name, j, used_var, used_par, nodes)

        self.function_container.addFunction(fun)

        return fun

    def createFinalCost(self, name, j):
        u = self.getInput().getVars()
        if cs.depends_on(j, u):
            raise RuntimeError(f'final cost "{name}" must not depend on the input')
        return self.createCostFunction(name, j, nodes=self.nodes-1,)

    def createIntermediateCost(self, name, j, nodes=None):
        if nodes is None:
            nodes = range(self.nodes-1)

        return self.createCostFunction(name, j, nodes=nodes)    

    def removeCostFunction(self, name):

        # if self.debug_mode:
        #     self.logger.debug('Functions before removal: {}'.format(self.costfun_container))
        self.function_container.removeFunction(name)
        # if self.debug_mode:
        #     self.logger.debug('Function after removal: {}'.format(self.costfun_container))

    def removeConstraint(self, name):
        self.function_container.removeFunction(name)

    def setNNodes(self, n_nodes):
        self.nodes = n_nodes + 1 # todo because I decided so
        self.var_container.setNNodes(self.nodes)
        self.function_container.setNNodes(self.nodes)

    def getNNodes(self) -> int:
        return self.nodes

    def createProblem(self, solver_type=None, solver_plugin=None, opts=None):

        # this is to reset both the constraints and the cost functions everytime I create a problem
        self.var_container.clear()
        self.function_container.clear()

        # implement the abstract state variable with the current node
        self.var_container.build()

        # implement the constraints and the cost functions with the current node
        self.function_container.build()

        # get j, w, g
        j = self.function_container.getCostFImplSum()
        w = self.var_container.getVarImplList()
        g = self.function_container.getCnstrFList()
        p = self.var_container.getParameterList()

        # self.logger.debug('state var unraveled:', self.state_var_container.getVarImplList())
        # self.logger.debug('constraints unraveled:', cs.vertcat(*self. ...))
        # self.logger.debug('cost functions unraveled:', cs.vertcat(*self. ...))
        # self.logger.debug('cost function summed:', self.j)
        # self.logger.debug('----------------------------------------------------')

        # if self.debug_mode:
        #     self.logger.debug('cost fun: {}'.format(j))
        #     self.logger.debug('state variables: {}'.format(w))
        #     self.logger.debug('constraints: {}'.format(g))

        self.prob = {'f': j, 'x': w, 'g': g, 'p': p}

        if self.solver is None:
            if solver_type is None:
                solver_type = self.default_solver
            if solver_plugin is None:
                solver_plugin = self.default_solver_plugin
            if opts is None:
                opts = dict()

            self.solver = solver_type('solver', solver_plugin, self.prob, opts)

    def getProblem(self):
        return self.prob

    def setSolver(self, solver):
        self.solver = solver

    def getSolver(self):
        return self.solver

    def solveProblem(self):

        # t_start = time.time()
        if self.prob is None:
            self.logger.warning('Problem is not created. Nothing to solve!')
            return 0

        self.var_container.updateBounds()
        self.var_container.updateInitialGuess()
        self.var_container.updateParameters()

        w0 = self.var_container.getInitialGuessList()

        if self.debug_mode:
            self.logger.debug('Initial guess vector for variables: {}'.format(self.var_container.getInitialGuessList()))


        lbw = self.var_container.getBoundsMinList()
        ubw = self.var_container.getBoundsMaxList()

        lbg = self.function_container.getLowerBoundsList()
        ubg = self.function_container.getUpperBoundsList()

        p = self.var_container.getParameterValues()

        if self.debug_mode:

            j = self.function_container.getCostFImplSum()
            w = self.var_container.getVarImplList()
            g = self.function_container.getCnstrFList()

            self.logger.debug('================')
            self.logger.debug(f'len w: {w.shape}')
            self.logger.debug(f'len lbw: {len(lbw)}')
            self.logger.debug(f'len ubw: {len(ubw)}')
            self.logger.debug(f'len w0: {len(w0)}')
            self.logger.debug(f'len g: {g.shape}')
            self.logger.debug(f'len lbg: {len(lbg)}')
            self.logger.debug(f'len ubg: {len(ubg)}')
            self.logger.debug(f'len p: {p.shape}')


            # self.logger.debug('================')
            self.logger.debug(f'w: {w}')
            self.logger.debug(f'lbw: {lbw}')
            self.logger.debug(f'ubw: {ubw}')
            self.logger.debug(f'g: {g}')
            self.logger.debug(f'lbg: {lbg}')
            self.logger.debug(f'ubg: {ubg}')
            self.logger.debug(f'j: {j}')
            self.logger.debug(f'p: {p}')
        # t_to_set_up = time.time() - t_start
        # print('T to set up:', t_to_set_up)
        # t_start = time.time()

        self.sol = self.solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p)

        # t_to_solve = time.time() - t_start
        # print('T to solve:', t_to_solve)
        # t_start = time.time()

        if self.crash_if_suboptimal:
            if not self.solver.stats()['success']:
                raise Exception('Optimal solution NOT found.')

        w_opt = self.sol['x'].full().flatten()

        # split solution for each variable
        solution_dict = {name: np.zeros([var.shape[0], len(var.getNodes())]) for name, var in self.var_container.getVarAbstrDict(past=False).items()}

        pos = 0
        for node, val in self.var_container.getVarImplDict().items():
            if node == 'nNone':
                for name, var in val.items():
                    dim = var['var'].shape[0]
                    solution_dict[name][:, 0] = w_opt[pos:pos + dim]
                    pos = pos + dim
            else:
                for name, var in val.items():
                    node_number = int(node[node.index('n') + 1:])
                    var_nodes = self.var_container.getVarAbstrDict(past=False)[name].getNodes()
                    if node_number in var_nodes:
                        dim = var['var'].shape[0]
                        solution_dict[name][:, node_number - var_nodes[0]] = w_opt[pos:pos + dim]
                        pos = pos + dim



        # t_to_finish = time.time() - t_start
        # print('T to finish:', t_to_finish)
        self.__solution = solution_dict
        return self.__solution

    def getSolution(self):
        return self.__solution

    def getVariables(self, name=None):

        if name is None:
            var = self.var_container.getVarAbstrDict(past=False)
        else:
            var = self.var_container.getVarAbstrDict(past=False)[name]

        return var

    def getConstraints(self, name=None):

        if name is None:
            fun = self.function_container.getCnstrFDict()
        else:
            fun = self.function_container.getCnstrFDict()[name]

        return fun

    def evalFun(self, fun):
        """
        input: name of the function to evaluate
        return: fun evaluated at all nodes using the solution of horizon problem
        """

        if self.__solution is None:
            raise Exception('The solution of the horizon problem is not computed yet. Cannot evaluate function.')

        fun_to_evaluate = fun.getFunction()
        all_vars = list()
        for var_name, var_list in fun.getVariables().items():
            for var in var_list:
                # careful about ordering
                # todo this is very ugly, but what can I do (wanted to do it without the if)
                if isinstance(var, sv.SingleVariable):
                    all_vars.append(self.__solution[var_name])
                else:
                    all_vars.append(self.__solution[var_name][:, np.array(fun.getNodes()) + var.offset])

        all_pars = list()
        for par_name, par_list in fun.getParameters().items():
            for par in par_list:
                # careful about ordering
                # todo this is very ugly, but what can I do (wanted to do it without the if)
                if isinstance(par, sv.SingleParameter):
                    all_pars.append(self.var_container.getParameterValues(par_name))
                else:
                    all_pars.append(self.var_container.getParameterValues(par_name)[:, fun.getNodes()])

        fun_evaluated = fun_to_evaluate(*(all_vars+all_pars)).toarray()
        return fun_evaluated

    def scopeNodeVars(self, node):

        return self.var_container.getVarImplAtNode(node)

    def scopeNodeConstraints(self, node):
        return self.function_container.getCnstrFImplAtNode(node)

    def scopeNodeCostFunctions(self, node):
        return self.function_container.getCostFImplAtNode(node)

    def serialize(self):

        self.var_container.serialize()
        self.function_container.serialize()


        if self.prob:

            # self.prob.clear()
            # print('serializing f (type: {}): {}'.format(type(self.prob['f']), self.prob['f']))
            # print('serializing x (type: {}): {}'.format(type(self.prob['x']), self.prob['x']))
            # print('serializing g (type: {}): {}'.format(type(self.prob['g']), self.prob['g']))

            self.prob['f'] = self.prob['f'].serialize()
            self.prob['x'] = self.prob['x'].serialize()
            self.prob['g'] = self.prob['g'].serialize()

            # print('serialized f (type: {}): {}'.format(type(self.prob['f']), self.prob['f']))
            # print('serialized x (type: {}): {}'.format(type(self.prob['x']), self.prob['x']))
            # print('serialized g (type: {}): {}'.format(type(self.prob['g']), self.prob['g']))


        return self

    def deserialize(self):

        self.var_container.deserialize()
        self.function_container.deserialize()

        if self.prob:
            self.prob['f'] = cs.Sparsity.deserialize(self.prob['f']) if self.function_container.getNCostFun() == 0 else cs.SX.deserialize(self.prob['f'])
            self.prob['x'] = cs.SX.deserialize(self.prob['x'])
            self.prob['g'] = cs.Sparsity.deserialize(self.prob['g']) if self.function_container.getNCnstrFun() == 0 else cs.SX.deserialize(self.prob['g'])

            # print('deserializing f', self.prob['f'])
            # print('deserializing x', self.prob['x'])
            # print('deserializing g', self.prob['g'])

        return self

if __name__ == '__main__':
    pass