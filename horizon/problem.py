import time
import warnings

import casadi as cs
from numpy.core.fromnumeric import var
from horizon import functions as fc
from horizon import variables as sv
import numpy as np
import logging
import sys
import pickle
import horizon.misc_function as misc
from typing import Union, Dict, List
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
        self.var_container = sv.VariablesContainer(self.logger)
        self.function_container = fc.FunctionsContainer(self.logger)

        self.state_aggr = sv.StateAggregate()
        self.input_aggr = sv.InputAggregate()
        self.state_der: cs.SX = None
        self.dt = None

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
        var = self.var_container.setStateVar(name, dim, range(self.nodes))
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
        var = self.var_container.setInputVar(name, dim, range(self.nodes-1))
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
        if nodes is not None:
            nodes = misc.checkNodes(nodes, range(self.nodes))

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
        if nodes is None:
            nodes = range(self.nodes)

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

    def resetDynamics(self):
        self.state_der = None

    def setDt(self, dt):
        """
        Setter of the system dt used in the optimization problem.

        Args:
            dt: dt of the system
        """

        # checks on dt
        # todo check that each dt in list has one dimension only
        if isinstance(dt, List):
            print('EXPERIMENTAL: you are setting a vector of dt. Be careful!')
            if len(dt) != self.getNNodes() - 1:
                raise Exception('Wrong dimension of dt vector.')
        elif isinstance(dt, (cs.SX, int, float)):
            pass
        else:
            raise ValueError(f'dt of type: {type(dt)} is not supported.')

        self.dt = dt

    def getDt(self):
        """
        Getter of the system dt used in the optimization problem.

        Returns:
            instance of the dt

        """
        if self.dt is None:
            raise ValueError('dt not defined, have you called setDt?')
        return self.dt

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
        for var in self.var_container.getParList(offset=True):
            if cs.depends_on(f, var):
                used_par.append(var)

        return used_par

    def createConstraint(self, name: str,
                         g,
                         nodes: Union[int, Iterable] = None,
                         bounds=None) -> fc.Constraint:
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
        used_var = self._getUsedVar(g)  # these now are fucking lists!
        used_par = self._getUsedPar(g)

        if self.debug_mode:
            self.logger.debug(f'Creating Constraint Function "{name}": active in nodes: {nodes} using vars {used_var}')

        # create internal representation of a constraint
        fun = fc.Constraint(name, g, used_var, used_par, nodes, bounds)

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

    def createCost(self, name: str,
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
        return self.createCost(name, j, nodes=self.nodes - 1)

    def createIntermediateCost(self,
                               name: str,
                               j,
                               nodes: Union[int, Iterable] = None):
        """
        Create a Cost Function that can be active on all the nodes except the last one.

        Args:
            name: name of the function
            j: function
            nodes: nodes the function is active on. If not specified, the cost function is active on all the nodes except the last one

        Returns:
            instance of Function

        """

        if nodes is None:
            nodes = range(self.nodes - 1)

        return self.createCost(name, j, nodes=nodes)

    def createResidual(self, name: str,
                       j,
                       nodes: Union[int, Iterable] = None):
        """
        Create a Residual Function of the optimization problem.

        Args:
            name: name of the function
            j: function
            nodes: nodes the function is active on. If not specified, it is active on ALL the nodes.

        Returns:
            instance of Residual Function

        """
        if nodes is None:
            nodes = list(range(self.nodes))
        else:
            nodes = misc.checkNodes(nodes, range(self.nodes))

        used_var = self._getUsedVar(j)
        used_par = self._getUsedPar(j)

        if self.debug_mode:
            self.logger.debug(f'Creating Residual Function "{name}": active in nodes: {nodes}')

        fun = fc.ResidualFunction(name, j, used_var, used_par, nodes)

        self.function_container.addFunction(fun)

        return fun

    def createFinalResidual(self, name: str, j):
        """
        Create a Residual Function only active on the last node of the optimization problem.

        Args:
            name: name of the residual function
            j: function

        Returns:
            instance of Residual Function

        """
        u = self.getInput().getVars()
        if cs.depends_on(j, u):
            raise RuntimeError(f'final residual "{name}" must not depend on the input')
        return self.createResidual(name, j, nodes=self.nodes - 1)

    def createIntermediateResidual(self,
                                   name: str,
                                   j,
                                   nodes: Union[int, Iterable] = None):
        """
        Create a Residual Function that can be active on all the nodes except the last one.

        Args:
            name: name of the function
            j: function
            nodes: nodes the function is active on. If not specified, the function is active on all the nodes except the last one

        Returns:
            instance of Function

        """

        if nodes is None:
            nodes = range(self.nodes - 1)

        return self.createResidual(name, j, nodes=nodes)

    def removeVariable(self, name: str) -> bool:
        """
        remove the desired variable.

        Args:
            name: name of the cost function to be removed

        Returns: False if function name is not found.

        """
        # todo add also remove from variable instance not only name
        return self.var_container.removeVar(name)

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

    def getParameters(self, name: str = None):
        """
        Getter for a desired parameter of the optimization problem.

        Args:
            name: name of the desired parameter. If not specified, a dict with all the parameters is returned

        Returns:
            the desired parameter/s
        """
        par = self.var_container.getPar(name)

        return par

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

    def getCosts(self, name=None):
        """
        Getter for a desired constraint of the optimization problem.

        Args:
            name: name of the desired constraint. If not specified, a dict with all the constraint is returned

        Returns:
            the desired constraint/s
        """
        fun = self.function_container.getCost(name)


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
                # this is required because:
                # I retrieve from the solution values only the nodes that the function is active on.
                # suppose a function is active on nodes [3, 4, 5, 6].
                # suppose a variable is defined on nodes [0, 1, 2, 3, 4, 5, 6].
                #   its solution will be [a, b, c, d, e, f, g]. I'm interested in the values at position [3, 4, 5, 6]
                # suppose a variable is defined only on nodes [3, 4, 5, 6].
                #   its solution will be [a, b, c, d]. I'm interested in the values at position [0, 1, 2, 3], which corresponds to nodes [3, 4, 5, 6]
                node_index = [i for i in range(len(var.getNodes())) if var.getNodes()[i] in np.array(fun.getNodes()) + var.getOffset()]
                all_vars.append(solution[var_name][:, node_index])

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


    def toParameter(self, var_name):

        warnings.warn('EXPERIMENTAL FUNCTION: toParameter')
        # TODO isn't there a way to change just the variable and everything else changes accordingly?
        # check if name of variable exists
        if var_name not in self.getVariables().keys():
            raise Exception(f'variable {var_name} not recognized.')

        old_var = self.getVariables(var_name)
        old_name = old_var.getName()
        old_dim = old_var.getDim()
        old_nodes = old_var.getNodes()
        self.removeVariable(var_name)

        if isinstance(old_var, sv.Variable):
            par = self.createParameter(old_name, old_dim, old_nodes)
        elif isinstance(old_var, sv.SingleVariable):
            par = self.createSingleParameter(old_name, old_dim)

        # if the variable is also the dt, set new dt
        dt = self.getDt()
        if isinstance(dt, List):
            for i in range(len(dt)):
                if isinstance(dt[i], (sv.Variable, sv.SingleVariable)):
                    if dt[i].getName() == var_name:
                        dt[i] = par
        if isinstance(dt, (sv.Variable, sv.SingleVariable)):
            if var_name == self.getDt().getName():
                self.setDt(par)

        if var_name in [input_var.getName() for input_var in self.getInput()]:
            self.getInput().removeVariable(var_name)
            self.getInput().addVariable(par)

        if var_name in [state_var.getName() for state_var in self.getState()]:
            self.getState().removeVariable(var_name)
            self.getState().addVariable(par)

        # transform variable to parameter (delete var and create a equal parameter)
        # modify constraints on their core (correct?)
        for fun in self.getConstraints().values():
            for i in range(len(fun.vars)):
                var_index_to_remove = list()
                if fun.vars[i].getName() == var_name:
                    # get the right offset of the parameter, if present
                    var_index_to_remove.append(i)
                    fun.pars.append(par.getParOffset(fun.vars[i].getOffset()))

                for index in sorted(var_index_to_remove, reverse=True):
                    del fun.vars[index]

                fun._project()

        # modify constraints on their core (correct?)
        for fun in self.getCosts().values():
            for i in range(len(fun.vars)):
                var_index_to_remove = list()
                if fun.vars[i].getName() == var_name:
                    # get the right offset, if present
                    var_index_to_remove.append(i)
                    fun.pars.append(par.getParOffset(fun.vars[i].getOffset()))

                for index in sorted(var_index_to_remove, reverse=True):
                    del fun.vars[index]
                # if fun.vars[i].getName() == var_name:
                #     fun.vars[i] = par

                fun._project()


        # for fun in self.getConstraints().values():
        #     for fun_var in fun.getVariables():
        #         if fun_var.getName() == var_name:
        #             print(f'found reference of {var_name} in {fun.getName()}')
        #             f_name = fun.getName()
        #             f_nodes = fun.getNodes()
        #             self.removeConstraint(fun.getName())
        #             self.createConstraint(f_name, fun._f, f_nodes)




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
        raise Exception('serialize yet to implement')
        # self.var_container.serialize()
        self.function_container.serialize()
        if self.state_der is not None:
            self.state_der = self.state_der.serialize()

        return self

    def deserialize(self):
        """
        Deserialize this class. Used for loading it.

        Returns:
            instance of the deserialized class "Problem"
        """
        raise Exception('serialize yet to implement')
        # self.var_container.deserialize()
        self.function_container.deserialize()
        if self.state_der is not None:
            self.state_der = cs.SX.deserialize(self.state_der)

        return self

    def save(self):
        data = dict()
        
        data['n_nodes'] = self.getNNodes() - 1

        # save state variables
        data['state'] = list()
        for sv in self.getState():
            var_data = dict()
            var_data['name'] = sv.getName()
            var_data['size'] = sv.size1()
            var_data['lb'] = sv.getLowerBounds().flatten('F').tolist()
            var_data['ub'] = sv.getUpperBounds().flatten('F').tolist()
            var_data['initial_guess'] = sv.getInitialGuess().flatten('F').tolist()
            data['state'].append(var_data)

        # save input variables
        data['input'] = list()
        for sv in self.getInput():
            var_data = dict()
            var_data['name'] = sv.getName()
            var_data['size'] = sv.size1()
            var_data['lb'] = sv.getLowerBounds().flatten('F').tolist()
            var_data['ub'] = sv.getUpperBounds().flatten('F').tolist()
            var_data['initial_guess'] = sv.getInitialGuess().flatten('F').tolist()
            data['input'].append(var_data)

        # save parameters
        data['param'] = dict()
        for p in self.var_container.getParList():
            var_data = dict()
            var_data['name'] = p.getName()
            var_data['size'] = p.getDim()
            var_data['values'] = p.getValues().flatten('F').tolist()
            data['param'][var_data['name']] = var_data

        # save cost and constraints
        data['cost'] = dict()
        for f in self.function_container.getCost().values():
            f : fc.Function = f
            var_data = dict()
            var_data['name'] = f.getName()
            var_data['repr'] = str(f.getFunction())
            var_data['var_depends'] = [v.getName() for v in f.getVariables()]
            var_data['param_depends'] = [v.getName() for v in f.getParameters()]
            var_data['nodes'] = f.getNodes()
            var_data['function'] = f.getFunction().serialize()
            data['cost'][var_data['name']] = var_data

        data['constraint'] = dict()
        for f in self.function_container.getCnstr().values():
            f : fc.Function = f
            var_data = dict()
            var_data['name'] = f.getName()
            var_data['repr'] = str(f.getFunction())
            var_data['var_depends'] = [v.getName() for v in f.getVariables()]
            var_data['param_depends'] = [v.getName() for v in f.getParameters()]
            var_data['nodes'] = f.getNodes()
            var_data['function'] = f.getFunction().serialize()
            var_data['lb'] = f.getLowerBounds().flatten('F').tolist()
            var_data['ub'] = f.getUpperBounds().flatten('F').tolist()
            data['constraint'][var_data['name']] = var_data

        return data


def pickleable(obj):
    try:
        pickle.dumps(obj)
    except pickle.PicklingError:
        return False
    return True

if __name__ == '__main__':

    import pickle
    from transcriptions import transcriptor
    from horizon.solvers import Solver
    from horizon.utils import plotter
    import matplotlib.pyplot as plt


    N = 2
    dt = 0.01
    prob = Problem(N)
    x = prob.createStateVariable('x', 2)
    y = prob.createInputVariable('y', 2)
    x_prev = x.getVarOffset(-1)

    xdot = cs.vertcat(x)
    prob.setDynamics(xdot)

    cnsrt = prob.createConstraint('cnsrt', x_prev + y, nodes=range(5, 11), bounds=dict(lb=[0, 0], ub=[10, 10]))
    cost = prob.createIntermediateCost('cost', x*y)

    # print('before', prob.var_container._vars)
    # print('before', prob.var_container._pars)
    # print('before:', [elem.getFunction() for elem in prob.function_container._cnstr_container.values()])
    # print('before:', [elem.getFunction() for elem in prob.function_container._costfun_container.values()])

    for fun in prob.function_container._cnstr_container.values():
        print(f"does {fun._f} depends on {prob.var_container._vars['y']}: {cs.depends_on(fun._f, prob.var_container._vars['y'])}")

    prob.serialize()
    print('===PICKLING===')
    prob_serialized = pickle.dumps(prob)
    print('===DEPICKLING===')
    prob_new = pickle.loads(prob_serialized)
    prb = prob_new.deserialize()

    for fun in prb.function_container._cnstr_container.values():
        print(f"does {fun._f} depends on {prb.var_container._vars['y']}: {cs.depends_on(fun._f, prb.var_container._vars['y'])}")

    exit()

    transcriptor.Transcriptor.make_method('multiple_shooting', prb, dt)
    sol = Solver.make_solver('ipopt', prb, dt)
    sol.solve()
    solution = sol.getSolutionDict()


    exit()

    N = 10
    dt = 0.01
    prob = Problem(10)
    x = prob.createStateVariable('x', 1)
    y = prob.createVariable('y', 1, nodes=range(5, 11))

    cnsrt = prob.createConstraint('cnsrt', x+y, nodes=range(5, 11))

    xdot = cs.vertcat(x)
    prob.setDynamics(xdot)

    sol = Solver.make_solver('ipopt', prob, dt)
    sol.solve()
    solution = sol.getSolutionDict()

    print(solution)
    hplt = plotter.PlotterHorizon(prob, solution)
    hplt.plotVariables()
    hplt.plotFunctions()

    plt.show()

