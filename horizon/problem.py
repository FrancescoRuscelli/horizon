import time

import casadi as cs
from horizon import functions as fc
from horizon import variables as sv
import numpy as np
import logging
import sys
import pickle
import horizon.misc_function as misc
from typing import Union, Dict
# from horizon.type_doc import BoundsDict
from collections.abc import Iterable
import inspect


class Problem:
    """
    Main class of Horizon, a tool for dynamic optimization using the symbolic framework CASADI. It is a useful tool
    to describe and solve parametric non-linear problems for trajectory planning. It follows the structure of a
    generic shooting method, dividing the interval of planning [ta, tb] into a given number of shooting nodes,
    where each nodes contains decision variables (state, input ..) of the problem.

    Horizon greatly simplifies the description of the problem, allowing the user to work only with abstract definitions
    of variables and functions, which are internally managed, evaluated and projected along the optimization horizon
    by the framework.
    """

    # todo probably better to set logger, not logging_level
    def __init__(self, N: int, crash_if_suboptimal: bool = False, logging_level=logging.INFO):
        """
        Initialize the optimization problem.

        Args:
            N: number of INTERMEDIATE nodes (transitions) in the optimization horizon. IMPORTANT: the final node is automatically generated. The problem will have N+1 nodes.
            crash_if_suboptimal: returns an Error if the solver cannot find an optimal solution
            logging_level: accepts the level of logging from package logging (INFO, DEBUG, ...)
        """
        self.opts = None
        self.solver = None
        self.__solution = None
        self.sol = None  # store solution from solver
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

    def createStateVariable(self, name: str, dim: int) -> sv.StateVariable:
        """
        Create a State Variable active on ALL the N+1 nodes of the optimization problem.

        Args:
            name: name of the variable
            dim: dimension of the variable

        Returns:
            instance of the State Variable

        """
        if self.state_der is not None:
            raise RuntimeError('createStateVariable must be called *before* setDynamics')
        var = self.var_container.setStateVar(name, dim)
        self.state_aggr.addVariable(var)
        return var

    def createInputVariable(self, name: str, dim: int) -> sv.InputVariable:
        """
        Create an Input Variable active on all the nodes of the optimization problem except the final one. (Input is not defined on the last node)

        Args:
            name: name of the variable
            dim: dimension of the variable

        Returns:
            instance of Input Variable
        """
        var = self.var_container.setInputVar(name, dim)
        self.input_aggr.addVariable(var)
        return var

    def createSingleVariable(self, name: str, dim: int) -> sv.SingleVariable:
        """
        Create a node-independent Single Variable of the optimization problem. It is a single decision variable which is not projected over the horizon.

        Args:
            name: name of the variable
            dim: dimension of the variable

        Returns:
            instance of Single Variable
        """
        var = self.var_container.setSingleVar(name, dim)
        return var

    def createVariable(self, name: str, dim: int, nodes: Iterable = None) -> Union[sv.StateVariable, sv.SingleVariable]:
        """
        Create a generic Variable of the optimization problem. Can be specified over a desired portion of the horizon nodes.

        Args:
            name: name of the variable
            dim: dimension of the variable
            nodes: nodes the variables is defined on. If not specified, the variable created is a Single Variable.

        Returns:
            instance of Variable

        """
        var = self.var_container.setVar(name, dim, nodes)
        return var

    def createParameter(self, name: str, dim: int, nodes: Iterable = None) -> Union[
        sv.Parameter, sv.SingleParameter]:
        """
        Create a Parameter used in the optimization problem. Can be specified over a desired portion of the horizon nodes.
        Parameters are specified before building the problem and can be 'assigned' afterwards, before solving the problem.

        Args:
            name: name of the parameter
            dim: dimension of the parameter
            nodes: nodes the parameter is defined on. If not specified, the variable is created on all the nodes.

        Returns:
            instance of Parameter

        """
        par = self.var_container.setParameter(name, dim, nodes)
        return par

    def createSingleParameter(self, name: str, dim: int) -> sv.SingleParameter:
        """
        Create a node-independent Single Parameter used to solve the optimization problem. It is a single parameter which is not projected over the horizon.
        Parameters are specified before building the problem and can be 'assigned' afterwards, before solving the problem.

        Args:
            name: name of the parameter
            dim: dimension of the parameter

        Returns:
            instance of Single Parameter

        """
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
        """
        Getter for the aggregate State defined for the problem. The State contains, in order, all the State Variables created.
        Returns:
            instance of State
        """
        return self.state_aggr

    def getInput(self) -> sv.InputAggregate:
        """
        Getter for the aggregate Input defined for the problem. The Input contains, in order, all the Input Variables created.

        Returns:
            instance of Input
        """
        return self.input_aggr

    def setDynamics(self, xdot):
        """
        Setter of the system Dynamics used in the optimization problem.

        Args:
            xdot: derivative of the State describing the dynamics of the system
        """
        nx = self.getState().getVars().shape[0]
        if xdot.shape[0] != nx:
            raise ValueError(f'state derivative dimension mismatch ({xdot.shape[0]} != {nx})')
        self.state_der = xdot

    def getDynamics(self) -> cs.SX:
        """
        Getter of the system Dynamics used in the optimization problem.

        Returns:
            instance of the derivative of the state

        """
        if self.state_der is None:
            raise ValueError('dynamics not defined, have you called setDynamics?')
        return self.state_der

    def setInitialState(self, x0: Iterable):
        self.getState().setBounds(lb=x0, ub=x0, nodes=0)

    def getInitialState(self) -> np.array:
        lb, ub = self.getState().getBounds(node=0)
        if np.any(lb != ub):
            return None
        return lb

    def _getUsedVar(self, f: cs.SX) -> list:
        """
        Finds all the variable used by a given CASADI function

        Args:
            f: function to be checked

        Returns:
            list of used variable

        """
        used_var = list()
        for var in self.var_container.getVarList(offset=True):
            if cs.depends_on(f, var):
                used_var.append(var)

        return used_var

    def _getUsedPar(self, f) -> list:
        """
        Finds all the parameters used by a given CASADI function

        Args:
            f: function to be checked

        Returns:
            list of used parameters

        """
        used_par = list()
        for var in self.var_container.getParList():
            if cs.depends_on(f, var):
                used_par.append(var)

        return used_par

    def _getUsedVarImpl(self, fun, used_var_abstract):
        used_var_impl = list()
        for var in used_var_abstract:
            var_impl = var.getImpl(fun.getNodes())
            var_dim = var.getDim()
            # reshape them for all-in-one evaluation of function
            # this is getting all the generic variable x, even if the function has a slice of it x[0:2].
            # It will work. The casadi function takes from x only the right slices afterwards.
            # print(var_impl)
            var_impl_matrix = cs.reshape(var_impl, (var_dim, len(fun.getNodes())))
            # generic input --> row: dim // column: nodes
            # [[x_0_0, x_1_0, ... x_N_0],
            #  [x_0_1, x_1_1, ... x_N_1]]
            used_var_impl.append(var_impl_matrix)

        return used_var_impl

    def _getUsedParImpl(self, fun, used_par_abstract):

        used_par_impl = list()
        for par in used_par_abstract:
            par_impl = par.getImpl(fun.getNodes())
            par_dim = par.getDim()
            par_impl_matrix = cs.reshape(par_impl, (par_dim, len(fun.getNodes())))
            used_par_impl.append(par_impl_matrix)

        return used_par_impl

    def createConstraint(self, name: str,
                         g,
                         nodes: Union[int, Iterable] = None,
                         bounds=None):
        """
        Create a Constraint Function of the optimization problem.

        Args:
            name: name of the constraint
            g: constraint function
            nodes: nodes the constraint is active on. If not specified, the Constraint is active on ALL the nodes.
            bounds: bounds of the constraint. If not specified, the bounds are set to zero.

        Returns:
            instance of Constraint

        """
        if nodes is None:
            nodes = range(self.nodes)
        else:
            nodes = misc.checkNodes(nodes, range(self.nodes))

        # get vars that constraint depends upon
        used_var = self._getUsedVar(g) # these now are fucking list!
        used_par = self._getUsedPar(g)

        if self.debug_mode:
            self.logger.debug(f'Creating Constraint Function "{name}": active in nodes: {nodes} using vars {used_var}')

        # create internal representation of a constraint
        fun = fc.Constraint(name, g, used_var, used_par, nodes, bounds)

        used_var_impl = self._getUsedVarImpl(fun, used_var)
        used_par_impl = self._getUsedParImpl(fun, used_par)

        fun._project(used_var_impl + used_par_impl)

        self.function_container.addFunction(fun)

        return fun

    def createFinalConstraint(self, name: str,
                              g,
                              bounds=None):
        """
        Create a Constraint Function only active on the last node of the optimization problem.

        Args:
            name: name of the constraint
            g: constraint function
            bounds: bounds of the constraint. If not specified, the bounds are set to zero.

        Returns:
            instance of Constraint

        """
        u = self.getInput().getVars()
        if cs.depends_on(g, u):
            raise RuntimeError(f'final constraint "{name}" must not depend on the input')
        return self.createConstraint(name, g, nodes=self.nodes - 1, bounds=bounds)

    def createIntermediateConstraint(self, name: str,
                                     g,
                                     nodes: Union[int, Iterable] = None,
                                     bounds=None):
        """
        Create a Constraint Function that can be active on all the nodes except the last one

        Args:
            name: name of the constraint
            g: constraint function
            nodes: nodes the constraint is active on. If not specified, the constraint is active on all the nodes except the last one
            bounds: bounds of the constraint. If not specified, the bounds are set to zero.

        Returns:
            instance of Constraint

        """
        if nodes is None:
            nodes = range(self.nodes - 1)
        return self.createConstraint(name, g, nodes=nodes, bounds=bounds)

    def createCostFunction(self, name: str,
                           j,
                           nodes: Union[int, Iterable] = None):
        """
        Create a Cost Function of the optimization problem.

        Args:
            name: name of the cost function
            j: cost function
            nodes: nodes the cost function is active on. If not specified, the cost function is active on ALL the nodes.

        Returns:
            instance of Cost Function

        """
        if nodes is None:
            nodes = range(self.nodes)
        else:
            nodes = misc.checkNodes(nodes, range(self.nodes))

        used_var = self._getUsedVar(j)
        used_par = self._getUsedPar(j)

        if self.debug_mode:
            self.logger.debug(f'Creating Cost Function "{name}": active in nodes: {nodes}')

        fun = fc.CostFunction(name, j, used_var, used_par, nodes)

        used_var_impl = self._getUsedVarImpl(fun, used_var)
        used_par_impl = self._getUsedParImpl(fun, used_par)

        fun._project(used_var_impl + used_par_impl)

        self.function_container.addFunction(fun)

        return fun

    def createFinalCost(self, name: str, j):
        """
        Create a Cost Function only active on the last node of the optimization problem.

        Args:
            name: name of the cost function
            j: cost function

        Returns:
            instance of Cost Function

        """
        u = self.getInput().getVars()
        if cs.depends_on(j, u):
            raise RuntimeError(f'final cost "{name}" must not depend on the input')
        return self.createCostFunction(name, j, nodes=self.nodes - 1, )

    def createIntermediateCost(self,
                               name: str,
                               j,
                               nodes: Union[int, Iterable] = None):
        """
        Create a Cost Function that can be active on all the nodes except the last one.

        Args:
            name: name of the cost function
            j: cost function
            nodes: nodes the cost function is active on. If not specified, the cost function is active on all the nodes except the last one

        Returns:
            instance of Cost Function

        """

        if nodes is None:
            nodes = range(self.nodes - 1)

        return self.createCostFunction(name, j, nodes=nodes)

    def removeCostFunction(self, name: str) -> bool:
        """
        remove the desired cost function.

        Args:
            name: name of the cost function to be removed

        Returns: False if function name is not found.

        """
        # if self.debug_mode:
        #     self.logger.debug('Functions before removal: {}'.format(self.costfun_container))
        return self.function_container.removeFunction(name)
        # if self.debug_mode:
        #     self.logger.debug('Function after removal: {}'.format(self.costfun_container))

    def removeConstraint(self, name: str) -> bool:
        """
        remove the desired constraint.

        Args:
            name: name of the constraint function to be removed

        Returns: False if function name is not found.

        """
        return self.function_container.removeFunction(name)

    def setNNodes(self, n_nodes: int):
        """
        set a desired number of nodes of the optimization problem.

        Args:
            n_nodes: new number of nodes

        """
        self.nodes = n_nodes + 1  # todo because I decided so
        self.var_container.setNNodes(self.nodes)
        self.function_container.setNNodes(self.nodes)

    def getNNodes(self) -> int:
        """
        Getter for the number of nodes of the optimization problem.

        Returns:
            the number of optimization nodes
        """
        return self.nodes

    def getVariables(self, name: str = None):
        """
        Getter for a desired variable of the optimization problem.

        Args:
            name: name of the desired variable. If not specified, a dict with all the variables is returned

        Returns:
            the desired variable/s
        """
        var = self.var_container.getVar(name)

        return var

    def getConstraints(self, name=None):
        """
        Getter for a desired constraint of the optimization problem.

        Args:
            name: name of the desired constraint. If not specified, a dict with all the constraint is returned

        Returns:
            the desired constraint/s
        """
        fun = self.function_container.getCnstr(name)


        return fun

    def evalFun(self, fun: fc.Function, solution):
        """
        Evaluates a given function over the solution found.

        Args:
            fun: function to evaluate

        Returns:
            fun evaluated at all nodes using the solution of horizon problem
        """
        fun_to_evaluate = fun.getFunction()
        all_vars = list()

        for var in fun.getVariables():
            var_name = var.getName()
            # careful about ordering
            # todo this is very ugly, but what can I do (wanted to do it without the if)
            if isinstance(var, sv.SingleVariable):
                all_vars.append(solution[var_name])
            else:
                all_vars.append(solution[var_name][:, np.array(fun.getNodes()) + var.getOffset()])

        all_pars = list()
        for par in fun.getParameters():
                # careful about ordering
                # todo this is very ugly, but what can I do (wanted to do it without the if)
            if isinstance(par, sv.SingleParameter):
                all_pars.append(par.getValues())
            else:
                par_matrix = np.reshape(par.getValues(), (par.getDim(), len(par.getNodes())), order='F')
                all_pars.append(par_matrix[:, fun.getNodes()])

        fun_evaluated = fun_to_evaluate(*(all_vars + all_pars)).toarray()
        return fun_evaluated

    def scopeNodeVars(self, node: int):
        """
        Scope the variables active at the desired node of the optimization problem.

        Args:
            node: desired node to scope

        Returns:
            all the active variables at the desired node
        """
        raise Exception('scopeNodeVars yet to be re-implemented')
        return self.var_container.getVarImplAtNode(node)

    def scopeNodeConstraints(self, node):
        """
        Scope the constraints active at the desired node of the optimization problem.

        Args:
            node: desired node to scope

        Returns:
            all the active constraint at the desired node
        """
        raise Exception('scopeNodeConstraint yet to be re-implemented')
        return self.function_container.getCnstrFImplAtNode(node)

    def scopeNodeCostFunctions(self, node):
        """
        Scope the cost functions active at the desired node of the optimization problem.

        Args:
            node: desired node to scope

        Returns:
            all the active cost functions at the desired node
        """
        raise Exception('scopeNodeCostFunctinos yet to be re-implemented')
        return self.function_container.getCostFImplAtNode(node)

    def serialize(self):
        """
        Serialize this class. Used for saving it.

        Returns:
            instance of the serialized class "Problem"
        """
        raise Exception('serialize yet to be re-implemented')
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
        """
        Deserialize this class. Used for loading it.

        Returns:
            instance of the deserialized class "Problem"
        """
        raise Exception('deserialize yet to be re-implemented')
        self.var_container.deserialize()
        self.function_container.deserialize()

        if self.prob:
            self.prob['f'] = cs.Sparsity.deserialize(
                self.prob['f']) if self.function_container.getNCostFun() == 0 else cs.SX.deserialize(self.prob['f'])
            self.prob['x'] = cs.SX.deserialize(self.prob['x'])
            self.prob['g'] = cs.Sparsity.deserialize(
                self.prob['g']) if self.function_container.getNCnstrFun() == 0 else cs.SX.deserialize(self.prob['g'])

            # print('deserializing f', self.prob['f'])
            # print('deserializing x', self.prob['x'])
            # print('deserializing g', self.prob['g'])

        return self


if __name__ == '__main__':
    N = 10
    dt = 0.01
    prob = Problem(10)
    x = prob.createStateVariable('x', 2)
    y = prob.createStateVariable('y', 4)
    u = prob.createInputVariable('u', 2)

    state = prob.getState()

    print(state)
    print(state.getVarIndex('x'))
    print(state.getVars())
    state_prev = state.getVarOffset(-1)
    print(state_prev.getVars())
    print(state_prev.getVarIndex('y'))

    pass
