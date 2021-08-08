import time

import casadi as cs
from horizon import function as fc
from horizon import state_variables as sv
import numpy as np
import logging
import sys
import pickle
import horizon.misc_function as misc
from typing import Union, Dict
from horizon.type_doc import BoundsDict
from collections.abc import Iterable

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
            nodes: nodes the parameter is defined on. If not specified, the variable created is a Single Parameter.

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
        return self.state_der

    def _getUsedVar(self, f) -> Dict[str, list]:
        """
        Finds all the variable used by a given CASADI function

        Args:
            f: function to be checked

        Returns:
            dictionary of used variable {name, list of variables}

        """
        used_var = dict()
        for name_var, value_var in self.var_container.getVarAbstrDict().items():
            for var in value_var:
                if cs.depends_on(f, var):
                    if name_var not in used_var:
                        used_var[name_var] = list()
                    used_var[name_var].append(var)

        return used_var

    def _getUsedPar(self, f) -> Dict[str, list]:
        """
        Finds all the parameters used by a given CASADI function

        Args:
            f: function to be checked

        Returns:
            dictionary of used parameters {name, list of parameters}

        """
        used_par = dict()
        for name_par, value_par in self.var_container.getParAbstrDict().items():
            if cs.depends_on(f, value_par):
                if name_par not in used_par:
                    used_par[name_par] = list()
                used_par[name_par].append(value_par)

        return used_par

    def createConstraint(self, name: str,
                         g,
                         nodes: Union[int, Iterable] = None,
                         bounds: BoundsDict = None):
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
        used_var = self._getUsedVar(g)
        used_par = self._getUsedPar(g)

        if self.debug_mode:
            self.logger.debug(f'Creating Constraint Function "{name}": active in nodes: {nodes} using vars {used_var}')

        # create internal representation of a constraint
        fun = fc.Constraint(name, g, used_var, used_par, nodes, bounds)

        self.function_container.addFunction(fun)

        return fun

    def createFinalConstraint(self, name: str,
                              g,
                              bounds: BoundsDict = None):
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
                                     bounds: BoundsDict = None):
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

    def createProblem(self, solver_type=None, solver_plugin: str = None, opts=None):
        """
        Generates the optimization problem, projecting all the variable and the function along the nodes, and initialize the solver.
        For more information on the solver, check https://web.casadi.org/python-api/#nlp
        If a solver is already set (using setSolver), that solver will be used.

        Args:
            solver_type: the desired solved. If not specified, 'nlpsol' is used
            solver_plugin: the desired solver plugin. if not specified, 'ipopt' is used
            opts: options used by the solver

        Returns:
            the created problem in the CASADI form: {'f': --, 'x': --, 'g': --, 'p': --}
        """
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

        return self.prob

    def getProblem(self):
        """
        Getter of the created problem.
        Returns:
            the created problem in the CASADI form: {'f': --, 'x': --, 'g': --, 'p': --}
        """
        return self.prob

    def setSolver(self, solver):
        """
        Set a custom solver for the optimization problem.
        Args:
            solver: the desired solver to be used.
        """
        self.solver = solver

    def getSolver(self):
        """
        Get the solver used in the optimization problem.

        Returns:
            instance of the solver
        """
        return self.solver

    def solveProblem(self) -> Union[bool, dict]:
        """
        Solves the problem after updating the bounds, the initial guesses and the parameters of the problem.

        Returns:
            the solution of the problem
        """
        # t_start = time.time()
        if self.prob is None:
            self.logger.warning('Problem is not created. Nothing to solve!')
            return False

        self.var_container.updateBounds()
        self.var_container.updateInitialGuess()
        self.var_container.updateParameters()

        w0 = self.var_container.getInitialGuessList()

        if self.debug_mode:
            self.logger.debug('Initial guess vector for variables: {}'.format(self.var_container.getInitialGuessList()))

        lbw = self.var_container.getLowerBoundsList()
        ubw = self.var_container.getUpperBoundsList()

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
        solution_dict = {name: np.zeros([var.shape[0], len(var.getNodes())]) for name, var in
                         self.var_container.getVarAbstrDict(offset=False).items()}

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
                    var_nodes = self.var_container.getVarAbstrDict(offset=False)[name].getNodes()
                    if node_number in var_nodes:
                        dim = var['var'].shape[0]
                        solution_dict[name][:, node_number - var_nodes[0]] = w_opt[pos:pos + dim]
                        pos = pos + dim

        # t_to_finish = time.time() - t_start
        # print('T to finish:', t_to_finish)
        self.__solution = solution_dict
        return self.__solution

    def getSolution(self) -> dict:
        """
        Getter for the ordered solution of the problem.
                * keys -> the name of the decision variables
                * values -> the values of the decision variables over the nodes

        Returns:
            the solution of the problem as a dict



        """
        return self.__solution

    def getVariables(self, name: str = None):
        """
        Getter for a desired variable of the optimization problem.

        Args:
            name: name of the desired variable. If not specified, a dict with all the variables is returned

        Returns:
            the desired variable/s
        """
        if name is None:
            var = self.var_container.getVarAbstrDict(offset=False)
        else:
            var = self.var_container.getVarAbstrDict(offset=False)[name]

        return var

    def getConstraints(self, name=None):
        """
        Getter for a desired constraint of the optimization problem.

        Args:
            name: name of the desired constraint. If not specified, a dict with all the constraint is returned

        Returns:
            the desired constraint/s
        """
        if name is None:
            fun = self.function_container.getCnstrFDict()
        else:
            fun = self.function_container.getCnstrFDict()[name]

        return fun

    def evalFun(self, fun: fc.Function):
        """
        Evaluates a given function over the solution found.

        Args:
            fun: function to evaluate

        Returns:
            fun evaluated at all nodes using the solution of horizon problem
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
        return self.var_container.getVarImplAtNode(node)

    def scopeNodeConstraints(self, node):
        """
        Scope the constraints active at the desired node of the optimization problem.

        Args:
            node: desired node to scope

        Returns:
            all the active constraint at the desired node
        """
        return self.function_container.getCnstrFImplAtNode(node)

    def scopeNodeCostFunctions(self, node):
        """
        Scope the cost functions active at the desired node of the optimization problem.

        Args:
            node: desired node to scope

        Returns:
            all the active cost functions at the desired node
        """
        return self.function_container.getCostFImplAtNode(node)

    def serialize(self):
        """
        Serialize this class. Used for saving it.

        Returns:
            instance of the serialized class "Problem"
        """
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
    nodes = 10

    prb = Problem(nodes, crash_if_suboptimal=True)
    x = prb.createStateVariable('x', 1)

    print(x.getImpl())
    pass
