import casadi as cs
from collections import OrderedDict
import logging
import numpy as np
import pickle
import horizon.misc_function as misc
import pprint

'''
now the StateVariable is only abstract at the very beginning.
Formerly
'''
# todo create function checker to check if nodes are in self.nodes and if everything is ok with the input (no dict, no letters...)

class AbstractVariable(cs.SX):
    """
    Abstract Variable of Horizon Problem.

    Notes:
          Horizon allows the user to work only with abstract variables. Internally, these variables are projected over the horizon nodes.
    """
    def __init__(self, tag: str, dim:int):
        """
        Initialize the Abstract Variable. Inherits from the symbolic CASADI varaible SX.

        Args:
            tag: name of the variable
            dim: dimension of the variable
        """
        super(AbstractVariable, self).__init__(cs.SX.sym(tag, dim))

        self.tag = tag
        self.dim = dim
        # offset of a variable is used to point to the desired previous/next implemented
        # Example:
            # offset 1 of variable x --> refers to implemented variable x at the next node
        self.offset = 0

    def getDim(self) -> int:
        """
        Getter for the dimension of the abstract variable.

        Returns:
            dimension of the variable
        """
        return self.dim

class SingleParameter(AbstractVariable):
    """
    Single Parameter of Horizon Problem.
    It is used for parametric problems: it is a symbolic variable in the optimization problem but it is not optimized. Rather, it is kept parametric and can be assigned before solving the problem.
    Parameters are specified before building the problem and can be 'assigned' afterwards, before solving the problem.
    The assigned value is the same along the horizon, since this parameter is node-independent.
    The Parameter is abstract, and gets implemented automatically.
    """
    def __init__(self, tag, dim, dummy_nodes):
        """
        Initialize the Single Parameter: a node-independent parameter which is not projected over the horizon.

        Args:
            tag: name of the parameter
            dim: dimension of the parameter
            dummy_nodes: useless input, used to simplify the framework mechanics
        """
        super(SingleParameter, self).__init__(tag, dim)

        self.par_impl = dict()
        self.par_impl['par'] = cs.SX.sym(self.tag + '_impl', self.dim)
        self.par_impl['val'] = np.zeros(self.dim)

    def assign(self, vals):
        """
        Assign a value to the parameter. Can be assigned also after the problem is built, before solving the problem.
        If not assigned, its default value is zero.

        Args:
            vals: value of the parameter
        """
        vals = misc.checkValueEntry(vals)

        # todo what if vals has no shape?
        if vals.shape[0] != self.dim:
            raise Exception('Wrong dimension of parameter values inserted.')

        self.par_impl['val'] = vals

    def getImpl(self, node=None):
        """
        Getter for the implemented parameter. Node is useless, since this parameter is node-independent.

        Args:
            node: useless input, used to simplify the framework mechanics

        Returns:
            instance of the implemented parameter
        """
        return self.par_impl['par']

    def getNodes(self):
        """
        Getter for the active nodes.

        Returns:
            -1 since this parameter is node-independent
        """
        # todo what if I return all the nodes?
        return [-1]

    def getValue(self, dummy_node):
        """
        Getter for the value assigned to the parameter. It is the same troughout all the nodes, since this parameter is node-independent.

        Args:
            dummy_node: useless input, used to simplify the framework mechanics

        Returns:
            value assigned to the parameter.
        """
        return self.par_impl['val']


class Parameter(AbstractVariable):
    """
    Parameter of Horizon Problem.
    It is used for parametric problems: it is a symbolic variable in the optimization problem but it is not optimized. Rather, it is kept parametric and can be assigned before solving the problem.
    """
    def __init__(self, tag, dim, nodes):
        """
        Initialize the Parameter: an abstract parameter projected over the horizon.
        Parameters are specified before building the problem and can be 'assigned' afterwards, before solving the problem.
        The assigned value can vary along the horizon, since this parameter is node-dependent.
        The Parameter is abstract, and gets implemented automatically.

        Args:
            tag: name of the parameter
            dim: dimension of the parameter
            nodes: nodes the parameter is implemented at
        """
        super(Parameter, self).__init__(tag, dim)

        self.par_impl = dict()

        self.nodes = nodes
        self._project()

    def _project(self):
        """
        Implements the parameter along the horizon nodes.
        Generates an ordered dictionary containing the implemented parameter and its value at each node {node: {par, val}}
        """
        new_par_impl = OrderedDict()

        for n in self.nodes:
            if 'n' + str(n) in self.par_impl:
                new_par_impl['n' + str(n)] = self.par_impl_dict['n' + str(n)]
            else:
                par_impl = cs.SX.sym(self.tag + '_' + str(n), self.dim)
                new_par_impl['n' + str(n)] = dict()
                new_par_impl['n' + str(n)]['par'] = par_impl
                new_par_impl['n' + str(n)]['val'] = np.zeros(self.dim)

        self.par_impl = new_par_impl

    def getNodes(self):
        """
        Getter for the nodes of the parameters.

        Returns:
            list of nodes the parameter is active on.
        """
        return self.nodes

    def assign(self, vals, nodes):
        """
       Assign a value to the parameter at a desired node. Can be assigned also after the problem is built, before solving the problem.
       If not assigned, its default value is zero.

       Args:
           vals: value of the parameter
           nodes: nodes at which the parameter is assigned
       """
        if nodes is None:
            nodes = self.nodes
        else:
            nodes = misc.checkNodes(nodes, self.nodes)

        vals = misc.checkValueEntry(vals)

        if vals.shape[0] != self.dim:
            raise Exception('Wrong dimension of parameter values inserted.')

        for node in nodes:
            self.par_impl['n' + str(node)]['val'] = vals

    def getImpl(self, node=None):
        """
        Getter for the implemented parameter.

        Args:
            node: node at which the parameter is retrieved. If not specified, this function returns an SX array with all the implemented parameters along the nodes.

        Returns:
            implemented instances of the abstract parameter
        """
        if node is None:
            vals = cs.vertcat([self.par_impl['n' + str(i)]['par'] for i in self.nodes])
        else:
            vals = self.par_impl['n' + str(node)]['par']

        return vals

    def getValue(self, node=None):
        """
        Getter for the value of the parameter.

        Args:
            node: node at which the value of the parameter is retrieved. If not specified, this function returns a matrix with all the values of the parameter along the nodes.

        Returns:
            value/s of the parameter
        """
        if node is None:
            vals = np.zeros([self.dim, len(self.nodes)])
            for dim in range(self.dim):
                vals[dim, :] = np.hstack([self.par_impl['n' + str(i)]['val'][dim] for i in self.nodes])
        else:
            vals = self.par_impl['n' + str(node)]['val']

        return vals

    def __reduce__(self):
        """
        Experimental function to serialize this element.

        Returns:
            instance of this element serialized
        """
        return (self.__class__, (self.tag, self.dim, self.nodes,))



class SingleVariable(AbstractVariable):
    """
    Single Variable of Horizon Problem: generic variable of the optimization problem.
    The single variable is the same along the horizon, since it is node-independent.
    The Variable is abstract, and gets implemented automatically.
    """
    def __init__(self, tag, dim, dummy_nodes):
        """
        Initialize the Single Variable: a node-independent variable which is not projected over the horizon.
        The bounds of the variable are initialized to -inf/inf.

        Args:
            tag: name of the variable
            dim: dimension of the variable
            dummy_nodes: useless input, used to simplify the framework mechanics
        """
        super(SingleVariable, self).__init__(tag, dim)

        self.var_impl = dict()
        # todo do i create another var or do I use the SX var inside SingleVariable?
        self.var_impl['var'] = cs.SX.sym(self.tag + '_impl', self.dim)
        self.var_impl['lb'] = np.full(self.dim, -np.inf)
        self.var_impl['ub'] = np.full(self.dim, np.inf)
        self.var_impl['w0'] = np.zeros(self.dim)

    def setLowerBounds(self, bounds):
        """
        Setter for the lower bounds of the variable.

        Args:
            bounds: value of the lower bounds
        """
        bounds = misc.checkValueEntry(bounds)

        if bounds.shape[0] != self.dim:
            raise Exception('Wrong dimension of lower bounds inserted.')

        self.var_impl['lb'] = bounds

    def setUpperBounds(self, bounds):
        """
        Setter for the upper bounds of the variable.

        Args:
            bounds: value of the upper bounds
        """
        bounds = misc.checkValueEntry(bounds)

        if bounds.shape[0] != self.dim:
            raise Exception('Wrong dimension of upper bounds inserted.')

        self.var_impl['ub'] = bounds

    def setBounds(self, lb, ub):
        """
        Setter for the bounds of the variable.

        Args:
            lb: value of the lower bounds
            ub: value of the upper bounds
        """
        self.setLowerBounds(lb)
        self.setUpperBounds(ub)

    def setInitialGuess(self, val):
        """
        Setter for the initial guess of the variable.

        Args:
            val: value of the initial guess
        """
        val = misc.checkValueEntry(val)

        if val.shape[0] != self.dim:
            raise Exception('Wrong dimension of initial guess inserted.')

        self.var_impl['w0'] = val

    def getImpl(self, dummy_node):
        """
        Getter for the implemented variable. Node is useless, since this variable is node-independent.

        Args:
            dummy_node: useless input, used to simplify the framework mechanics

        Returns:
            implemented instances of the abstract variable
        """
        var_impl = self.var_impl['var']
        return var_impl

    def _getVals(self, val_type, dummy_node):
        """
        wrapper function to get the desired argument from the variable.

        Args:
            val_type: type of the argument to retrieve
            dummy_node: if None, returns an array of the desired argument

        Returns:
            value/s of the desired argument
        """
        if dummy_node is None:
            # returns the value in the form of an array
            vals = np.array([self.var_impl[val_type]]).T
        else:
            vals = self.var_impl[val_type]

        return vals

    def getLowerBounds(self, dummy_node=None):
        """
        Getter for the lower bounds of the variable.

        Args:
            node: useless input, used to simplify the framework mechanics

        Returns:
            values of the lower bounds

        """
        return self._getVals('lb', dummy_node)

    def getUpperBounds(self, dummy_node=None):
        """
        Getter for the upper bounds of the variable.

        Args:
            node: useless input, used to simplify the framework mechanics

        Returns:
            values of the upper bounds

        """
        return self._getVals('ub', dummy_node)

    def getBounds(self, dummy_node=None):
        """
        Getter for the bounds of the variable.

        Args:
            node: useless input, used to simplify the framework mechanics

        Returns:
            values of the bounds

        """
        return self.getLowerBounds(dummy_node), self.getUpperBounds(dummy_node)

    def getInitialGuess(self, dummy_node=None):
        """
        Getter for the initial guess of the variable.

        Args:
            node: useless input, used to simplify the framework mechanics

        Returns:
            values of the initial guess
        """
        return self._getVals('w0', dummy_node)

    def getNodes(self):
        """
        Getter for the active nodes of the variable.

        Returns:
            -1 since this parameter is node-independent
        """
        # todo what if I return all the nodes?
        return [-1]

    def getVarOffsetDict(self):
        """
        Getter for the offset variables. Useless, since this variable is node-independent.

        Returns:
            empty dict
        """
        return dict()

    def getImplDim(self):
        """
        Getter for the dimension of the implemented variables.

        Returns:
            dimension of the variable
        """
        return self.shape[0]

class Variable(AbstractVariable):
    """
    Variable of Horizon Problem: generic variable of the optimization problem.
    The Variable is abstract, and gets implemented automatically over the horizon nodes.

    Examples:
        Abstract variable "x". Horizon nodes are N.

        Implemented variable "x" --> x_0, x_1, ... x_N-1, x_N
    """
    def __init__(self, tag, dim, nodes):
        """
        Initialize the Variable.
        The bounds of the variable are initialized to -inf/inf.

        Args:
            tag: name of the variable
            dim: dimension of the variable
            nodes: nodes the variable is defined on
        """
        super(Variable, self).__init__(tag, dim)

        self.nodes = nodes

        # self.var = cs.SX.sym(tag, dim)
        self.var_offset = dict()
        self.var_impl = dict()

        # i project the variable over the optimization nodes
        self._project()

    def setLowerBounds(self, bounds, nodes=None):
        """
        Setter for the lower bounds of the variable.

        Args:
            bounds: desired bounds of the variable
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        if nodes is None:
            nodes = self.nodes
        else:
            nodes = misc.checkNodes(nodes, self.nodes)

        bounds = misc.checkValueEntry(bounds)
        if bounds.shape[0] != self.dim:
            raise Exception('Wrong dimension of lower bounds inserted.')

        for node in nodes:
            self.var_impl['n' + str(node)]['lb'] = bounds

    def setUpperBounds(self, bounds, nodes=None):
        """
        Setter for the upper bounds of the variable.

        Args:
            bounds: desired bounds of the variable
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        if nodes is None:
            nodes = self.nodes
        else:
            nodes = misc.checkNodes(nodes, self.nodes)

        bounds = misc.checkValueEntry(bounds)

        if bounds.shape[0] != self.dim:
            raise Exception('Wrong dimension of upper bounds inserted.')

        for node in nodes:
            self.var_impl['n' + str(node)]['ub'] = bounds

    def setBounds(self, lb, ub, nodes=None):
        """
        Setter for the bounds of the variable.

        Args:
            lb: desired lower bounds of the variable
            ub: desired upper bounds of the variable
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def setInitialGuess(self, val, nodes=None):
        """
        Setter for the initial guess of the variable.

        Args:
            val: desired initial guess of the variable
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        if nodes is None:
            nodes = self.nodes
        else:
            nodes = misc.checkNodes(nodes, self.nodes)

        val = misc.checkValueEntry(val)

        if val.shape[0] != self.dim:
            raise Exception('Wrong dimension of initial guess inserted.')

        for node in nodes:
            self.var_impl['n' + str(node)]['w0'] = val

    def getVarOffset(self, node):
        """
        Getter for the offset variable. An offset variable is used to point to the desired implemented instance of the abstract variable.

        Examples:
            Abstract variable "x". Horizon nodes are N.

            Implemented variable "x" --> x_0, x_1, ... x_N-1, x_N

            Offset variable "x-1" points FOR EACH NODE at variable "x" implemented at the PREVIOUS NODE.

        Args:
            val: desired initial guess of the variable
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        if node > 0:
            node = f'+{node}'

        if node in self.var_offset:
            return self.var_offset[node]
        else:

            createTag = lambda name, node: name + str(node) if node is not None else name

            new_tag = createTag(self.tag, node)
            var = AbstractVariable(new_tag, self.dim)
            var.offset = int(node)

            self.var_offset[node] = var
        return var

    def getVarOffsetDict(self):
        """
        Getter for the offset variables.

        Returns:
            dict with all the offset variables referring to this abstract variable
        """
        return self.var_offset

    def _setNNodes(self, n_nodes):
        """
        set a desired number of nodes to the variable.

        Args:
            n_nodes: the desired number of nodes to be set
        """
        # todo this is because I must manage Variable, InputVariable, StateVariable in different ways.
        self.nodes = n_nodes
        self._project()

    def _project(self):
        """
        Implements the variable along the horizon nodes.
        Generates an ordered dictionary containing the implemented variables and its value at each node {node: {var, lb, ub, w0}}
        """
        new_var_impl = OrderedDict()

        for n in self.nodes:
            if 'n' + str(n) in self.var_impl:
                # when reprojecting, if the implemented variable is present already, use it. Do not create a new one.
                new_var_impl['n' + str(n)] = self.var_impl['n' + str(n)]
            else:
                var_impl = cs.SX.sym(self.tag + '_' + str(n), self.dim)
                new_var_impl['n' + str(n)] = dict()
                new_var_impl['n' + str(n)]['var'] = var_impl
                new_var_impl['n' + str(n)]['lb'] = np.full(self.dim, -np.inf)
                new_var_impl['n' + str(n)]['ub'] = np.full(self.dim, np.inf)
                new_var_impl['n' + str(n)]['w0'] = np.zeros(self.dim)

        self.var_impl = new_var_impl

    def getImpl(self, node=None):
        """
        Getter for the implemented variable.

        Args:
            node: node at which the variable is retrieved

        TODO:
            If not specified, this function should return an SX array with all the implemented variables along the nodes.

        Returns:
            implemented instances of the abstract variable
        """
        # todo this is another option: reproject everytime one asks for .getImpl
        # var_impl = self._projectN(node)
        if node is None:
            var_impl = cs.vertcat(*[self.var_impl['n' + str(i)]['var'] for i in self.nodes])
        else:
            var_impl = self.var_impl['n' + str(node)]['var']
        return var_impl

    def _getVals(self, val_type, node):
        """
        wrapper function to get the desired argument from the variable.

        Args:
            val_type: type of the argument to retrieve
            node: desired node at which the argument is retrieved. If not specified, this returns the desired argument at all nodes

        Returns:
            value/s of the desired argument
        """
        if node is None:
            vals = np.zeros([self.shape[0], len(self.nodes)])
            for dim in range(self.shape[0]):
                vals[dim, :] = np.hstack([self.var_impl['n' + str(i)][val_type][dim] for i in self.nodes])
        else:
            vals = self.var_impl['n' + str(node)][val_type]
        return vals

    def getLowerBounds(self, node=None):
        """
        Getter for the lower bounds of the variable.

        Args:
            node: desired node at which the lower bounds are retrieved. If not specified, this returns the lower bounds at all nodes

        Returns:
            value/s of the lower bounds

        """
        return self._getVals('lb', node)

    def getUpperBounds(self, node=None):
        """
        Getter for the upper bounds of the variable.

        Args:
            node: desired node at which the upper bounds are retrieved. If not specified, this returns the upper bounds at all nodes

        Returns:
            value/s of the upper bounds

        """
        return self._getVals('ub', node)

    def getBounds(self, node=None):
        """
        Getter for the bounds of the variable.

        Args:
            node: desired node at which the bounds are retrieved. If not specified, this returns the bounds at all nodes

        Returns:
            value/s of the bounds

        """
        return self.getLowerBounds(node), self.getUpperBounds(node)

    def getInitialGuess(self, node=None):
        """
        Getter for the initial guess of the variable.

        Args:
            node: desired node at which the initial guess is retrieved. If not specified, this returns the lower bounds at all nodes

        Returns:
            value/s of the bounds

        """
        return self._getVals('w0', node)

    def getImplDim(self):
        """
        Getter for the dimension of the implemented variables, considering all the nodes.

        Returns:
            dimension of the variable multiplied by number of nodes
        """
        return self.shape[0] * len(self.getNodes())

    def getNodes(self):
        """
        Getter for the active nodes of the variable.

        Returns:
            the nodes the variable is defined on
        """
        return self.nodes

    def __reduce__(self):
        """
        Experimental function to serialize this element.

        Returns:
            instance of this element serialized
        """
        return (self.__class__, (self.tag, self.dim, self.nodes, ))


class InputVariable(Variable):
    """
    Input (Control) Variable of Horizon Problem.
    The variable is abstract, and gets implemented automatically over the horizon nodes except the last one.

    Examples:
        Abstract variable "x". Horizon nodes are N.

        Implemented variable "x" --> x_0, x_1, ... x_N-1
    """
    def __init__(self, tag, dim, nodes):
        """
        Initialize the Input Variable.

        Args:
            tag: name of the variable
            dim: dimension of the variable
            nodes: should always be N-1, where N is the number of horizon nodes
        """
        super(InputVariable, self).__init__(tag, dim, nodes)

class StateVariable(Variable):
    """
    State Variable of Horizon Problem.
    The variable is abstract, and gets implemented automatically over all the horizon nodes.

    Examples:
        Abstract variable "x". Horizon nodes are N.

        Implemented variable "x" --> x_0, x_1, ... x_N-1, x_N
    """
    def __init__(self, tag, dim, nodes):
        """
        Initialize the State Variable.

        Args:
            tag: name of the variable
            dim: dimension of the variable
            nodes: should always be N, where N is the number of horizon nodes
        """
        super(StateVariable, self).__init__(tag, dim, nodes)

class AbstractAggregate():
    """
    Abstract Aggregate of the Horizon Problem.
    Used to store more variables of the same nature.
    """
    def __init__(self, *args: AbstractVariable):
        """
        Initialize the Abstract Aggregate.

        Args:
            *args: abstract variables of the same nature
        """
        self.var_list = [item for item in args]

    def getVars(self) -> cs.SX:
        """
        Getter for the variable stored in the aggregate.

        Returns:
            a SX vector of all the variables stored
        """
        return cs.vertcat(*self.var_list)

    def __iter__(self):
        """
        Aggregate can be treated as an iterable.
        """
        yield from self.var_list

    def __getitem__(self, ind):
        """
        Aggregate can be accessed with indexing.
        """
        return self.var_list[ind]

class Aggregate(AbstractAggregate):
    """
    Aggregate of the Horizon Problem.
    Used to store more variables of the same nature.
    """
    def __init__(self, *args):
        """
        Initialize the Aggregate.

        Args:
            *args: instances of abstract variables of the same nature
        """
        super().__init__(*args)

    def getVarOffset(self, offset):
        """
        Getter for the offset variables contained in the aggregate.

        Returns:
            an abstract aggregate with all the offset variables referring to the relative abstract variables
        """
        var_list = list()
        for var in self.var_list:
            var_list.append(var.getVarOffset(offset))

        return AbstractAggregate(*var_list)

    def addVariable(self, var):
        """
        Adds a Variable to the Aggregate.

        Todo:
            Should check if variable type belongs to the aggregate type (no mixing!)

        Args:
            var: variable to be added to the aggregate
        """
        self.var_list.append(var)

    def setBounds(self, lb, ub, nodes=None):
        """
        Setter for the bounds of the variables in the aggregate.

        Args:
            lb: desired lower bounds of the variable
            ub: desired upper bounds of the variable
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def setLowerBounds(self, lb, nodes=None):
        """
        Setter for the lower bounds of the variables in the aggregate.

        Args:
            bounds: list of desired bounds of the all the variables in the aggregate
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        idx = 0
        for var in self:
            nv = var.shape[0]
            var.setLowerBounds(lb[idx:idx+nv], nodes)
            idx += nv

    def setUpperBounds(self, ub, nodes=None):
        """
        Setter for the upper bounds of the variables in the aggregate.

        Args:
            bounds: list of desired bounds of the all the variables in the aggregate
            nodes: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes
        """
        idx = 0
        for var in self:
            nv = var.shape[0]
            var.setUpperBounds(ub[idx:idx+nv], nodes)
            idx += nv

    def getBounds(self, node):
        """
        Getter for the bounds of the variables in the aggregate.

        Args:
            node: which nodes the bounds are applied on. If not specified, the variable is bounded along ALL the nodes

        Returns:
            array of bound values of each variable in the aggregate

        todo:
            test this!
        """
        lb = self.getLowerBounds(node)
        ub = self.getUpperBounds(node)

        return lb, ub

    def getLowerBounds(self, node):
        """
        Getter for the lower bounds of the variables in the aggregate.

        Args:
            node: which nodes the lower bounds are applied on. If not specified, the variable is bounded along ALL the nodes

        Returns:
            array of lower bound values of each variable in the aggregate

        todo:
            test this!
        """
        return np.hstack((var.getLowerBounds(node) for var in self))

    def getUpperBounds(self, node):
        """
        Getter for the upper bounds of the variables in the aggregate.

        Args:
            node: which nodes the upper bounds are applied on. If not specified, the variable is bounded along ALL the nodes

        Returns:
            array of upper bound values of each variable in the aggregate

        todo:
            test this!
        """
        return np.hstack((var.getUpperBounds(node) for var in self))

class StateAggregate(Aggregate):
    """
    State Aggregate of the Horizon Problem.
    Used to store all the state variables.
    """
    def __init__(self, *args: StateVariable):
        """
        Initialize the State Aggregate.

        Args:
            *args: instances of state variables
        """
        super().__init__(*args)

class InputAggregate(Aggregate):
    """
    Input (Control) Aggregate of the Horizon Problem.
    Used to store all the control variables.
    """
    def __init__(self, *args: InputVariable):
        """
        Initialize the Input (Control) Aggregate.

        Args:
            *args: instances of input (control) variables
        """
        super().__init__(*args)

# todo what if this is completely useless? at the end of the day, I need this Container for:
#   .getVarAbstrDict() --> get all abstract variables (special attention for the past variables)
#   .getVarImpl(): ---> get implemented variable at node
#   .getVarImplList() ---> get all the implemented variables as list
#   .getVarImplDict() ---> get all the implemented variables as dict
#   Can I do something else? Right now .build() orders them like as follows:
#            (nNone: [vars..]
#                n0: [vars..],
#                n1: [vars..], ...)
#   but since order is everything I care about, is there a simpler way?
#   for var in self.vars:
#    all_impl += var.getAllImpl
#   this is ordered with the variables and I probably don't need build?
#            (x: [n0, n1, n2 ...]
#             y: [n0, n1, n2, ...]
#             z: [nNone, n0, n1, ...])

#
class VariablesContainer:
    """
    Container of all the variables of Horizon.
    It is used internally by the Problem to get the abstract and implemented variables.
    """
    def __init__(self, nodes, logger=None):
        """
        Initialize the Variable Container.

        Args:
           nodes: the number of nodes of the problem
           logger: a logger reference to log data
        """
        self.logger = logger
        self.nodes = nodes

        self.vars = OrderedDict()
        self.vars_impl = OrderedDict()

        self.pars = OrderedDict()
        self.pars_impl = OrderedDict()

    def createVar(self, var_type, name, dim, active_nodes):
        """
        Create a variable and adds it to the Variable Container.

        Args:
            var_type: type of variable
            name: name of variable
            dim: dimension of variable
            active_nodes: nodes the variable is defined on
        """
        if active_nodes is not None:
            active_nodes = misc.checkNodes(active_nodes, range(self.nodes))

        var = var_type(name, dim, active_nodes)
        self.vars[name] = var

        if self.logger:
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug('Creating variable {} as {}'.format(name, var_type))

        return var

    def setVar(self, name, dim, active_nodes=None):
        """
        Creates a generic variable.

        Args:
            name: name of the variable
            dim: dimension of the variable
            active_nodes: nodes the variable is defined on. If not specified, a Single Variable is generated
        """
        if active_nodes is None:
            var_type = SingleVariable
        else:
            var_type = Variable

        var = self.createVar(var_type, name, dim, active_nodes)
        return var

    def setStateVar(self, name, dim):
        """
        Creates a State variable.

        Args:
            name: name of the variable
            dim: dimension of the variable
        """
        var = self.createVar(StateVariable, name, dim, range(self.nodes))
        return var

    def setInputVar(self, name, dim):
        """
        Creates a Input (Control) variable.

        Args:
            name: name of the variable
            dim: dimension of the variable
        """
        var = self.createVar(InputVariable, name, dim, range(self.nodes-1))
        return var

    def setSingleVar(self, name, dim):
        """
        Creates a Single variable.

        Args:
            name: name of the variable
            dim: dimension of the variable
        """
        var = self.createVar(SingleVariable, name, dim, None)
        return var

    def setParameter(self, name, dim, nodes):
        """
        Creates a Parameter.

        Args:
            name: name of the variable
            dim: dimension of the variable
            nodes: nodes the parameter is defined on. If not specified, all the horizon nodes are considered
        """
        if nodes is None:
            nodes = range(self.nodes)

        par = Parameter(name, dim, nodes)
        self.pars[name] = par

        if self.logger:
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug(f'Creating parameter "{name}"')

        return par

    def setSingleParameter(self, name, dim):
        """
        Creates a Single Variable.

        Args:
            name: name of the variable
            dim: dimension of the variable
        """
        par = SingleParameter(name, dim, None)
        self.pars[name] = par

        if self.logger:
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug(f'Creating single parameter "{name}"')

        return par

    def getParameterList(self):
        """
        Getter for the Parameters in the Variable Container. Ordered following the nodes order.

        Returns:
            a list of all the abstract parameters
        """
        # self.pars_impl --> {node: {parameter_name: par, val}}
        par_impl_list = list()
        # for each node
        for node in self.pars_impl.values():
            # for each parameter in node
            for parameter in node.keys():
                # get from state_var_impl the relative var
                par_impl_list.append(node[parameter]['par'])

        return cs.vertcat(*par_impl_list)

    def getParameterValues(self, name=None):
        """
        Getter for the assigned values to the desired Parameter in the Variable Container. Ordered following the nodes order.

        Args:
            name: name of the desired parameter. If not specified, returns the values of the parameter along all the nodes

        Returns:
            values of the parameters
        """
        # self.pars_impl --> {node: {parameter_name: par, val}}
        par_impl_list = list()
        if name is None:
            # for each node
            for node in self.pars_impl.values():
                for parameter in node.keys():
                    # get from state_var_impl the relative var
                    par_impl_list.append(node[parameter]['val'])

            return cs.vertcat(*par_impl_list)

        else:
            # todo can I do this elsewhere in getPar and getVar? it's nice!
            return self.pars[name].getValue()

    def getVarsDim(self):
        """
        Getter for the total dimension of the variables in the Variable Container (dim * number of nodes).

        Returns:
            total dimension of the variables
        """
        var_dim_tot = 0
        for var in self.vars.values():
            var_dim_tot += var.getImplDim()
        return var_dim_tot

    def getStateVars(self):
        """
        Getter for the state variables in the Variable Container.

        Returns:
            a dict with all the state variables
        """
        state_vars = dict()
        for name, var in self.vars.items():
            if isinstance(var, StateVariable):
                state_vars[name] = var

        return state_vars

    def getInputVars(self):
        """
        Getter for the input (control) variables in the Variable Container.

        Returns:
            a dict with all the input (control) variables
        """
        input_vars = dict()
        for name, var in self.vars.items():
            if isinstance(var, InputVariable):
                input_vars[name] = var

        return input_vars

    def getVarAbstrDict(self, offset=True):
        """
        Getter for the abstract variables in the Variable Container. Used by the Horizon Problem.

        Args:
            offset: if True, get also the offset_variable

        Returns:
            a dict with all the abstract variables
        """
        if offset:
            var_abstr_dict = dict()
            for name, var in self.vars.items():
                var_abstr_dict[name] = list()
                var_abstr_dict[name].append(var)
                for var_offset in var.getVarOffsetDict().values():
                    var_abstr_dict[name].append(var_offset)
        else:
            # todo beware of deep copy
            var_abstr_dict = self.vars

        return var_abstr_dict

    def getParAbstrDict(self):
        """
        Getter for the abstract parameters in the Variable Container. Used by the Horizon Problem.

        Returns:
           a dict with all the abstract parameters
        """
        # todo beware of deep copy
        return self.pars

    def getParImpl(self, name, k):
        """
         Getter for the desired implemented parameter at the desired node in the Variable Container. Used by the Horizon Problem.

        todo:
            this should be a variation of getParameterList or getParameterValues

        Args:
            name: name of the desired parameter
            k: node the parameter is implemented at

        Returns:
            instance of the implemented parameter
        """

        if isinstance(self.pars[name], SingleParameter):
            k = None

        node_name = 'n' + str(k)

        if node_name in self.pars_impl:
            par = self.pars_impl[node_name][name]['par']
        else:
            par = None

        return par

    def getVarImpl(self, name, k):
        """
        Getter for the desired implemented variable at the desired node in the Variable Container. Used by the Horizon Problem.

        todo:
            this should be a variation of getVarDict or getVarList

        Args:
            name: name of the desired variable
            k: node the variable is implemented at

        Returns:
            instance of the implemented variable
        """
        if isinstance(self.vars[name], SingleVariable):
            k = None

        node_name = 'n' + str(k)

        if node_name in self.vars_impl:
            var = self.vars_impl[node_name][name]['var']
        else:
            var = None

        return var

    def getVarImplAtNode(self, k):
        """
        Getter for all the implemented variables at a desired node.

        todo:
            should be embedded in getVarImpl
        Args:
            k: desired node to scope

        Returns:
            dict of all the variable at desired node
        """
        if 'n' + str(k) in self.vars_impl:
            return self.vars_impl['n' + str(k)]
        else:
            return None

    def getVarImplDict(self):
        """
        Getter for all the implemented variables.

        todo:
            should be embedded in getVarImpl or vice-versa
        Returns:
            dict with all nodes and relative implemented variables
        """
        return self.vars_impl

    def getVarImplList(self):
        """
        Getter for the list of all implemented variables. Used by Horizon Problem in buildProblem to retrieve the final vector of optimization variables.

        todo:
            should be embedded in getVarImpl or vice-versa

        Returns:
            SX vector (vertcat) of all the implemented variables at each node

        """

        state_var_impl_list = list()
        for val in self.vars_impl.values():
            for var_abstract in val.keys():
                # get from state_var_impl the relative var

                state_var_impl_list.append(val[var_abstract]['var'])

        return cs.vertcat(*state_var_impl_list)

    def _getVarInfoList(self, var_type):
        """
        Getter for the desired information of a variable.

        Args:
            var_type: type of argument to be retrieved

        Returns:
            array of the desired propery along the horizon nodes
        """
        state_var_bound_list = np.zeros(self.getVarsDim())

        j = 0
        for node, val in self.vars_impl.items():
            for var_abstract in val.keys():
                var = val[var_abstract][var_type]
                dim = val[var_abstract][var_type].shape[0]
                state_var_bound_list[j:j + dim] = var
                j = j + dim

        return state_var_bound_list

    def getLowerBoundsList(self):
        """
        Getter for all the lower bounds of all the variables.
        Used by Horizon Problem in solveProblem to retrieve the final vector of lower bounds.

        Returns:
            an array containing all the lower bound values
        """
        return self._getVarInfoList('lb')

    def getUpperBoundsList(self):
        """
        Getter for all the upper bounds of all the variables.
        Used by Horizon Problem in solveProblem to retrieve the final vector of upper bounds.

        Returns:
            an array containing all the upper bound values
        """
        return self._getVarInfoList('ub')

    def getInitialGuessList(self):
        """
        Getter for all the initial bounds of all the variables.
        Used by Horizon Problem in solveProblem to retrieve the final vector of upper bounds.

        Returns:
            an array containing all the initial values
        """
        return self._getVarInfoList('w0')

    def _fillVar(self, name, node, val):
        """
        Fills a dict with the implemented variable and its properties (lb, ub, w0) at a given node.
        self.vars_impl is a dict {node: {name: var, lb, ub, w0}}

        Args:
            name: name of the abstract variable
            node: desired node
            val: value of the variable (only required for logging)
        """
        # todo bounds are not necessary here
        # node can be None ---> nNone contains all the single variables
        node_name = 'n' + str(node)
        var_impl = self.vars[name].getImpl(node)
        var_bound_min = self.vars[name].getLowerBounds(node)
        var_bound_max = self.vars[name].getUpperBounds(node)
        initial_guess = self.vars[name].getInitialGuess(node)
        var_dict = dict(var=var_impl, lb=var_bound_min, ub=var_bound_max, w0=initial_guess)
        self.vars_impl[node_name].update({name: var_dict})

        if self.logger:
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug('Implemented {} of type {}: {}'.format(name, type(val), var_impl))

    def _fillPar(self, name, node, val):
        """
        Fills a dict with the implemented parameters and its properties (par, val) at a given node.
        self.pars_impl is a dict {node: {name: par, val}}
        Args:
            name: name of the abstract parameter
            node: desired node
            val: value of the parameter (only required for logging)
        """
        # node can be None ---> nNone contains all the single parameters
        node_name = 'n' + str(node)
        par_impl = self.pars[name].getImpl(node)
        par_value = self.pars[name].getValue(node)
        par_dict = dict(par=par_impl, val=par_value)
        self.pars_impl[node_name].update({name: par_dict})

        if self.logger:
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug('Implemented {} of type {}: {}'.format(name, type(val), par_impl))

    def build(self):
        """
        fill the dictionary "state_var_impl"
            - key: nodes (nNone, n0, n1, ...) nNone contains single variables that are not projected in nodes
            - val: dict with name and value of implemented variable
        """

        # todo I'm tired now but I believe I can use directly the values from the Variables: maybe it is not necessary
        #  to create a self.vars_impl? VariableContainer may become just a mirror searching for the values that all the abstract variables holds

        # todo not sure about the timings, but here a dictionary with potentially MANY empty value is created.
        # contains single vars that do not need to be projected
        self.vars_impl['nNone'] = dict()
        self.pars_impl['nNone'] = dict()

        # for each node, instantiate a dict() ..
        for node in range(self.nodes):
            # which contains variables that are projected along the nodes
            self.vars_impl['n' + str(node)] = dict()

            for name, val in self.vars.items():

                # if it is a single variable, search at nNode (key of the self.vars_impl and self.vars[name])
                if isinstance(val, SingleVariable):
                    self._fillVar(name, None, val)
                    continue

                if node in self.vars[name].getNodes():
                    self._fillVar(name, node, val)

        # same thing for parameters
        for node in range(self.nodes):
            self.pars_impl['n' + str(node)] = dict()
            for name, val in self.pars.items():

                if isinstance(val, SingleParameter):
                    self._fillPar(name, None, val)
                    continue

                if node in self.pars[name].getNodes():
                    self._fillPar(name, node, val)

    def updateBounds(self):
        """
        Updates the bounds of each variable in Variable Container.
        Used by Horizon Problem in solveProblem to get the final vector of all bounds.
        It asks each abstract variables (in self.vars) for its bounds and set them to self.vars_impl
        """
        for node in self.vars_impl.keys():
            for name, state_var in self.vars_impl[node].items():
                k = node[node.index('n') + 1:]
                state_var['lb'] = self.vars[name].getLowerBounds(k)
                state_var['ub'] = self.vars[name].getUpperBounds(k)

    def updateInitialGuess(self):
        """
        Updates the initial guess of each variable in Variable Container.
        Used by Horizon Problem in solveProblem to get the final vector of all intial guesses.
        It asks each abstract variables (in self.vars) for its initial guess and set them to self.vars_impl
        """
        for node in self.vars_impl.keys():
            for name, state_var in self.vars_impl[node].items():
                k = node[node.index('n') + 1:]
                state_var['w0'] = self.vars[name].getInitialGuess(k)

    def updateParameters(self):
        """
        Updates the assigned value of each parameters in Variable Container.
        Used by Horizon Problem in solveProblem to get the final vector of all assigned parameter values.
        It asks each abstract variables (in self.pars) for its initial guess and set them to self.pars_impl
        """
        for node in self.pars_impl.keys():
            for name, parameter in self.pars_impl[node].items():
                k = node[node.index('n') + 1:]
                parameter['val'] = self.pars[name].getValue(k)

    def setNNodes(self, n_nodes):
        """
        set a desired number of nodes to the Variable Container.

        Args:
            n_nodes: the desired number of nodes to be set
        """
        # this is required to update the self.state_var_impl EACH time a new number of node is set
        # removed_nodes = [node for node in range(self.nodes) if node not in range(n_nodes)]
        # for node in removed_nodes:
        #     if 'n' + str(node) in self.state_var_impl:
        #         del self.state_var_impl['n' + str(node)]

        self.nodes = n_nodes
        for var in self.vars.values():
            if isinstance(var, SingleVariable):
                pass
            elif isinstance(var, InputVariable):
                var._setNNodes(list(range(self.nodes-1)))
            elif isinstance(var, StateVariable):
                var._setNNodes(list(range(self.nodes)))
            elif isinstance(var, Variable):
                var._setNNodes([node for node in var.getNodes() if node in list(range(self.nodes))])

        for par in self.pars.values():
            if isinstance(par, SingleParameter):
                pass
            elif isinstance(par, Parameter):
                par._setNNodes([node for node in var.getNodes() if node in list(range(self.nodes))])



    def clear(self):
        """
        Clears all the implemented variables.
        """
        self.vars_impl.clear()


    def serialize(self):
        """
        Serialize the Variable Container. Used to save it.

        Returns:
           instance of serialized Variable Container
        """
        # todo how to do? I may use __reduce__ but I don't know how
        # for name, value in self.state_var.items():
        #     print('state_var', type(value))
        #     self.state_var[name] = value.serialize()

        # for name, value in self.state_var_prev.items():
        #     print('state_var_prev', type(value))
        #     self.state_var_prev[name] = value.serialize()

        for node, item in self.vars_impl.items():
            for name, elem in item.items():
                self.vars_impl[node][name]['var'] = elem['var'].serialize()

    def deserialize(self):
        """
        Deserialize the Variable Container. Used to load it.

        Returns:
           instance of deserialized Variable Container
        """
        # for name, value in self.state_var.items():
        #     self.state_var[name] = cs.SX.deserialize(value)
        #
        # for name, value in self.state_var_prev.items():
        #     self.state_var_prev[name] = cs.SX.deserialize(value)

        for node, item in self.vars_impl.items():
            for name, elem in item.items():
                self.vars_impl[node][name]['var'] = cs.SX.deserialize(elem['var'])

    # def __reduce__(self):
    #     return (self.__class__, (self.nodes, self.logger, ))

if __name__ == '__main__':

    n_nodes = 10
    sv = VariablesContainer(n_nodes)
    x = sv.setStateVar('x', 2)

    exit()
    x = StateVariable('x', 2, 4)
    u = InputVariable('u', 2, 4)
    print(isinstance(u, StateVariable))


    exit()
    # x._project()
    # print('before serialization:', x)
    # print('bounds:', x.getBounds(2))
    # x.setBounds(2,2)
    # print('bounds:', x.getBounds(2))
    # print('===PICKLING===')
    # a = pickle.dumps(x)
    # print(a)
    # print('===DEPICKLING===')
    # x_new = pickle.loads(a)
    #
    # print(type(x_new))
    # print(x_new)
    # print(x_new.tag)
    #
    # print('bounds:', x.getBounds(2))
    # print(x.var_impl)
    # exit()

    # x = StateVariable('x', 2, 15)
    # print([id(val['var']) for val in x.var_impl.values()])
    # x._setNNodes(20)
    # print([id(val['var']) for val in x.var_impl.values()])

    n_nodes = 10
    sv = VariablesContainer(n_nodes)
    x = sv.setStateVar('x', 2)
    x_prev = x.createAbstrNode(-1)

    print(x_prev)
    print(type(x_prev))
    print(type(x))

    exit()
    sv.setStateVar('x', 2, -1)
    sv.setStateVar('y', 2)

    for k in range(n_nodes):
        sv.update(k)

    # print(sv.state_var)
    # print(sv.state_var_prev)
    pprint.pprint(sv.vars_impl)
    # pprint.pprint(sv.getVarAbstrDict())
    # pprint.pprint(sv.getVarImplDict())
    # pprint.pprint(sv.getVarImpl('x-1', 1))


    exit()
    # x_prev = sv.setVar('x', 2, -2)
    #
    # for i in range(4):
    #     sv.update(i)
    #
    # print('state_var_prev', sv.state_var_prev)
    # print('state_var_impl', sv.state_var_impl)
    #
    # print('sv.getVarAbstrDict()', sv.getVarAbstrDict())
    # print('sv.getVarAbstrList()', sv.getVarAbstrList())
    # print('sv.getVarImplList()', sv.getVarImplList())
    # print('sv.getVarImpl()', sv.getVarImpl('x-2', 0))

    print('===PICKLING===')
    sv_serialized = pickle.dumps(sv)
    print(sv_serialized)
    print('===DEPICKLING===')
    sv_new = pickle.loads(sv_serialized)

    print(sv_new.vars)
    print(sv_new.state_var_prev)
    print(sv_new.vars_impl)
    print(sv_new.getVarAbstrDict())
    print(sv_new.getVarImplDict())
