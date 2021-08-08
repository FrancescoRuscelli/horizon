import casadi as cs
import numpy as np
from horizon import misc_function as misc
from collections import OrderedDict
import pickle
import time
from typing import Union, Iterable
from horizon.type_doc import BoundsDict

class Function:

    """
        Function of Horizon: generic function of SX values from CASADI.

        Notes:
            This function is an abstract representation of its projection over the nodes of the optimization problem.
            An abstract function gets internally implemented at each node, using the variables at that node.
    """
    def __init__(self, name: str, f: cs.SX, used_vars: dict, used_pars: dict, nodes: Union[int, Iterable]):
        """
        Initialize the Horizon Function.

        Args:
            name: name of the function
            f: SX function
            used_vars: variable used in the function
            used_pars: parameters used in the function
            nodes: nodes the function is active on
        """

        self.f = f
        self.name = name
        self.nodes = []

        # get a list of all the variable used in the function -> dict(str:list)
        all_vars = list()
        for name, val in used_vars.items():
            for item in val:
                all_vars.append(item)

        # get a list of all the parameters used in the function -> dict(str:list)
        all_pars = list()
        for name, val in used_pars.items():
            for item in val:
                all_pars.append(item)


        # todo isn't there another way to get the variables from the function g?
        self.vars = used_vars
        self.pars = used_pars

        # create function of CASADI, dependent on (in order) [all_vars, all_pars]
        self.fun = cs.Function(name, all_vars + all_pars, [self.f])
        self.fun_impl = dict()

        self.setNodes(nodes)

    def getName(self) -> str:
        """
        Getter for the name of the function

        Returns:
            name of the function
        """
        return self.name

    def getDim(self) -> int:
        """
        Getter for the dimension of the function

        Returns:
            dimension of the function
        """
        return self.f.shape[0]

    def getFunction(self) -> cs.Function:
        """
        Getter for the CASADI function

        Returns:
            instance of the CASADI function
        """
        return self.fun

    def getFunctionImpl(self, node):
        """
        Getter for the CASADI function implemented at the desired node
        Args:
            node: the desired node of the implemented function to retrieve

        Returns:
            instance of the CASADI function at the desired node
        """
        if 'n' + str(node) in self.fun_impl:
            return self.fun_impl['n' + str(node)]
        else:
            return None

    def _project(self, node: int, used_vars: list):
        """
        Implements the function at the desired node using the desired variables.

        Args:
            node: the desired node at which the function is implemented
            used_vars: the variable used at the desired node

        Returns:
            the implemented function
        """
        # print('projected fun "{}" with vars {}:'.format(self.fun, used_vars))
        # todo are we 100% sure the order of the input variables is right?
        self.fun_impl['n' + str(node)] = self.fun(*used_vars)
        return self.fun_impl['n' + str(node)]

    def getNodes(self) -> list:
        """
        Getter for the active nodes of the function.

        Returns:
            a list of the nodes where the function is active

        """
        return self.nodes

    def setNodes(self, nodes, erasing=False):
        """
        Setter for the active nodes of the function.

        Args:
            nodes: list of desired active nodes.
            erasing: choose if the inserted nodes overrides the previous active nodes of the function. 'False' if not specified.
        """
        if erasing:
            self.nodes.clear()

        # adding active nodes to function nodes
        for i in nodes:
            if i not in self.nodes:
                self.nodes.append(i)
                self.nodes.sort()

    def getVariables(self) -> dict:
        """
        Getter for the variables used in the function.

        Returns:
            a dictionary of all the used variable. {name: cs.SX}
        """
        return self.vars

    def getParameters(self) -> dict:
        """
        Getter for the parameters used in the function.

        Returns:
            a dictionary of all the used parameters. {name: cs.SX}
        """
        return self.pars

    def getType(self) -> str:
        """
        Getter for the type of the Variable.

        Notes:
            Is it useful?

        Returns:
            a string describing the type of the function
        """
        return 'generic'

    def serialize(self):
        """
        Serialize the Function. Used to save it.

        Returns:
            serialized instance of Function
        """
        self.f = self.f.serialize()

        for name, data in self.vars.items():
            self.vars[name] = data.serialize()

        for node, item in self.fun_impl.items():
            self.fun_impl[node] = item.serialize()

        self.fun = self.fun.serialize()

        return self

    def deserialize(self):
        """
        Deserialize the Function. Used to load it.

        Returns:
            deserialized instance of Function
        """
        self.f = cs.SX.deserialize(self.f)

        for name, data in self.vars.items():
            self.vars[name] = cs.SX.deserialize(data)

        for node, item in self.fun_impl.items():
            self.fun_impl[node] = cs.Function.deserialize(item)

        self.fun = cs.Function.deserialize(self.fun)

        return self


class Constraint(Function):
    """
    Constraint Function of Horizon.
    """
    def __init__(self, name: str, f: cs.SX, used_vars: dict, used_pars: dict, nodes: Union[int, Iterable], bounds: BoundsDict =None):
        """
        Initialize the Constraint Function.

        Args:
            name: name of the constraint function
            f: constraint SX function
            used_vars: variable used in the function
            used_pars: parameters used in the function
            nodes: nodes the function is active on
            bounds: bounds of the constraint. If not specified, the bounds are set to zero.
        """
        self.bounds = dict()

        # constraints are initialize to 0.: 0. <= x <= 0.
        for node in nodes:
            self.bounds['n' + str(node)] = dict(lb=np.full(f.shape[0], 0.), ub=np.full(f.shape[0], 0.))

        super().__init__(name, f, used_vars, used_pars, nodes)

        # manage bounds
        if bounds is not None:

            if 'nodes' not in bounds:
                bounds['nodes'] = None
            # if only one constraint is set, we assume:
            if 'lb' not in bounds:  # -inf <= x <= ub
                bounds['lb'] = np.full(f.shape[0], -np.inf)
            else:
                # todo what if bounds does not have shape!
                if bounds['lb'].shape[0] != self.getDim():
                    raise Exception('Wrong dimension of lower bounds inserted.')

            if 'ub' not in bounds:  # lb <= x <= inf
                bounds['ub'] = np.full(f.shape[0], np.inf)
            else:
                if bounds['ub'].shape[0] != self.getDim():
                    raise Exception('Wrong dimension of upper bounds inserted.')

            self.setBounds(lb=bounds['lb'], ub=bounds['ub'], nodes=bounds['nodes'])

    # todo transform string in typeFun "ConstraintType"
    def getType(self) -> str:
        """
        Getter for the type of the Constraint.

        Returns:
            a string describing the type of the function
        """
        return 'constraint'

    def setLowerBounds(self, bounds, nodes=None):
        """
        Setter for the lower bounds of the function.

        Args:
            bounds: desired bounds of the function
            nodes: nodes of the function the bounds are applied on. If not specified, the function is bounded along ALL the nodes.
        """
        if nodes is None:
            nodes = self.nodes
        else:
            nodes = misc.checkNodes(nodes, self.nodes)

        bounds = misc.checkValueEntry(bounds)

        # todo what if bounds does not have shape!
        if bounds.shape[0] != self.getDim():
            raise Exception('Wrong dimension of lower bounds inserted.')

        for node in nodes:
            if node in self.nodes:
                self.bounds['n' + str(node)].update({'lb': bounds})

        # print('function lower bounds: {}'.format(nodes))

    def setUpperBounds(self, bounds, nodes=None):
        """
        Setter for the upper bounds of the function.

        Args:
            bounds: desired bounds of the function
            nodes: nodes of the function the bounds are applied on. If not specified, the function is bounded along ALL the nodes.
        """
        if nodes is None:
            nodes = self.nodes
        else:
            nodes = misc.checkNodes(nodes, self.nodes)

        bounds = misc.checkValueEntry(bounds)

        if bounds.shape[0] != self.getDim():
            raise Exception('Wrong dimension of upper bounds inserted.')

        for node in nodes:
            if node in self.nodes:
                self.bounds['n' + str(node)].update({'ub': bounds})

        # print('function upper bounds: {}'.format(nodes))

    def setBounds(self, lb, ub, nodes=None):
        """
        Setter for the bounds of the function.

        Args:
            lb: desired lower bounds of the function
            ub: desired upper bounds of the function
            nodes: nodes of the function the bounds are applied on. If not specified, the function is bounded along ALL the nodes.
        """
        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def _getVals(self, val_type: str, node: int):
        """
        wrapper function to get the desired argument from the constraint.

        Args:
            val_type: type of the argument to retrieve
            node: desired node at which the argument is retrieved. If not specified, this returns the desired argument at all nodes.

        Returns:
            value/s of the desired argument
        """
        if node is None:
            vals = np.zeros([self.f.shape[0], len(self.nodes)])
            for dim in range(self.f.shape[0]):
                vals[dim, :] = np.hstack([self.bounds['n' + str(i)][val_type][dim] for i in self.nodes])
        else:
            vals = self.bounds['n' + str(node)][val_type]
        return vals

    def getLowerBounds(self, node: int =None):
        """
        Getter for the lower bounds of the function.

        Args:
            node: desired node at which the lower bounds are retrieved. If not specified, this returns the lower bounds at all nodes.

        Returns:
            value/s of the lower bounds

        """
        lb = self._getVals('lb', node)
        return lb

    def getUpperBounds(self, node: int =None):
        """
        Getter for the upper bounds of the function.

        Args:
            node: desired node at which the upper bounds are retrieved. If not specified, this returns the upper bounds at all nodes.

        Returns:
            value/s of the upper bounds

        """
        ub = self._getVals('ub', node)
        return ub

    def getBounds(self, nodes=None):
        """
        Getter for the bounds of the function.

        Args:
            node: desired node at which the bounds are retrieved. If not specified, this returns the bounds at all nodes.

        Returns:
            value/s of the upper bounds
        """
        return self.getLowerBounds(nodes), self.getUpperBounds(nodes)

    def setNodes(self, nodes, erasing=False):
        """
        Setter for the active nodes of the constraint function.

        Args:
            nodes: list of desired active nodes.
            erasing: choose if the inserted nodes overrides the previous active nodes of the function. 'False' if not specified.
        """
        if erasing:
            self.nodes.clear()

        # adding to function nodes
        for i in nodes:
            if i not in self.nodes:
                self.nodes.append(i)
                self.nodes.sort()
                # for all the "new nodes" that weren't there, add default bounds
                if 'n' + str(i) not in self.bounds:
                    self.bounds['n' + str(i)] = dict(lb=np.full(self.f.shape[0], 0.),
                                                     ub=np.full(self.f.shape[0], 0.))

class CostFunction(Function):
    """
    Cost Function of Horizon.
    """
    def __init__(self, name, f, used_vars, used_pars, nodes):
        """
        Initialize the Cost Function.

        Args:
            name: name of the constraint function
            f: constraint SX function
            used_vars: variable used in the function
            used_pars: parameters used in the function
            nodes: nodes the function is active on
        """
        super().__init__(name, f, used_vars, used_pars, nodes)

    def getType(self):
        """
        Getter for the type of the Cost Function.

        Returns:
            a string describing the type of the function
        """
        return 'costfunction'

class FunctionsContainer:
    """
    Container of all the function of Horizon.
    It is used internally by the Problem to get the abstract and implemented function.

    Methods:
        build: builds the container with the updated functions.
    """
    def __init__(self, state_vars, nodes, logger=None):
        """
        Initialize the Function Container.

        Args:
            state_vars: all the decision variable used in the problem
            nodes: the number of nodes of the problem
            logger: a logger reference to log data
        """
        self.logger = logger

        # the Variable Container
        self.state_vars = state_vars

        # the number of nodes
        self.nodes = nodes

        # containers for the constraints (abstract and implemented)
        self.cnstr_container = OrderedDict()
        self.cnstr_impl = OrderedDict()

        # containers for the cost function (abstract and implemented)
        self.costfun_container = OrderedDict()
        self.costfun_impl = OrderedDict()

    def addFunction(self, fun: Function):
        """
        Add a function to the Function Container.

        Args:
            fun: a Function (can be Constraint or Cost Function) o add
        """
        if fun.getType() == 'constraint':
            self.cnstr_container[fun.getName()] = fun
        elif fun.getType() == 'costfunction':
            self.costfun_container[fun.getName()] = fun
        elif fun.getType() == 'generic':
            print('function.py: generic not implemented')

    def removeFunction(self, fun_name: str):
        """
        Remove a function from the Function Container.

        Args:
            fun_name: name of the function to be removed
        """
        if fun_name in self.cnstr_container:
            del self.cnstr_container[fun_name]
            return True
        elif fun_name in self.costfun_container:
            del self.costfun_container[fun_name]
            return True
        else:
            return False


    def getFunction(self, fun_name: str):
        """
        Getter for a Function inside the Function Container.
        Args:
            fun_name: name of the function to retrieve
        """
        if fun_name in self.cnstr_container:
            return self.cnstr_container[fun_name]
        elif fun_name in self.costfun_container:
            return self.costfun_container[fun_name]
        else:
            return None

    def getCnstrFDict(self) -> OrderedDict:
        """
        Getter for the dictionary of all the abstract constraint functions.
        Returns:
            ordered dict of the functions {name: fun}
        """
        return self.cnstr_container

    def getCostFDict(self) -> OrderedDict:
        """
        Getter for the dictionary of all the abstract cost functions.
        Returns:
            ordered dict of the functions {name: fun}
        """
        return self.costfun_container

    def build(self):
        """
        Builds the Function Container, updating all the variables.
        Updating consists in implementing all the abstract variables along each active nodes, using the variable at the corresponding node.
        """
        self.updateFun(self.cnstr_container, self.cnstr_impl)
        self.updateFun(self.costfun_container, self.costfun_impl)

    def updateFun(self, container, container_impl):
        """
        Implement the functions in 'container' and put them in 'container_impl'.

        Notes:
            container_impl is a nested dictionary {node: value}:
                - node: the node of the horizon problem
                - value: a dictionary with: {val: --, lb: --, ub: --}

        Args:
            container: container of abstract variables to implement
            container_impl: container where to put the implemented functions
        """

        # for each node in the horizon nodes
        for node in range(self.nodes):
            # initialize a dict for each node which is going to be populated by function values
            container_impl['n' + str(node)] = dict()

            # todo I really hate this implementation! Find a better one for used_vars
            # for each function in container (containing ABSTRACT functions)
            for fun_name, fun in container.items():
                # if the current node is in the active nodes of the function
                if node in fun.getNodes():
                    # initialize a list of used variable
                    used_vars = list()
                    # and for each ABSTRACT variable used by the function
                    for name, val in fun.getVariables().items():
                        for item in val:
                            # get the implemented variable at the right node (offset is necessary to account for past/next variables)
                            var = self.state_vars.getVarImpl(name, node + item.offset)
                            if var is None:
                                raise Exception(f'Variable out of scope: {item} does not exist at node {node}')
                            used_vars.append(var)

                    # same as above, but with parameters (in this case the notion of offset is not present)
                    used_pars = list()
                    for name, val in fun.getParameters().items():
                        for item in val:
                            var = self.state_vars.getParImpl(name, node)
                            if var is None:
                                raise Exception(f'Parameter out of scope: {item} does not exist at node {node}')
                            used_pars.append(var)

                    # finally, project the function at the active node using the required variables and the parameters
                    f_impl = fun._project(node, used_vars + used_pars)
                    if fun.getType() == 'constraint':
                        lb = fun.getLowerBounds(node)
                        ub = fun.getUpperBounds(node)
                        # and, if it's a constraint, add also lower and upper bounds
                        fun_dict = dict(val=f_impl, lb=lb, ub=ub)
                    else:
                        fun_dict = f_impl

                    # the container at node 'n' contains all the implemented function at node 'n' with the right variables
                    container_impl['n' + str(node)].update({fun_name: fun_dict})
                    # print('==================================================')
                    self.logger.debug(f'Implemented function "{fun_name}" of type {fun.getType()}: {f_impl} with vars {used_vars} at node {node}')

    def getCnstrDim(self) -> int:
        """
        Getter for the dimension of all the constraints
        Returns:
            the total dimension of the constraint
        """
        total_dim = 0
        for cnstr in self.cnstr_container.values():
            total_dim += cnstr.getDim() * len(cnstr.getNodes())

        return total_dim

    def getCnstrFImpl(self, name: str, node: int) -> Union[None, Constraint]:
        # todo this and all the methods below can be wrapped in a single function, depending on the arguments inserted
        """
        Getter for a desired Constraint implemented at a desired node.

        Args:
            name: the name of the desired constraint
            node: the node at which the desired constraint is implemented

        Returns:
            instance of Constraint Function if present, otherwise None
        """
        if 'n' + str(node) in self.cnstr_impl:
            if name in self.cnstr_impl['n' + str(node)]:
                return self.cnstr_impl['n' + str(node)][name]

        return None


    def getCostFImpl(self, name, node) -> Union[None, CostFunction]:
        """
        Getter for a desired Cost Function implemented at a desired node.

        Args:
            name: the name of the desired cost function
            node: the node at which the desired cost function is implemented

        Returns:
            instance of Cost Function if present, otherwise None
        """
        if 'n' + str(node) in self.costfun_impl:
            if name in self.costfun_impl['n' + str(node)]:
                return self.costfun_impl['n' + str(node)][name]

        return None

    def getCnstrFImplAtNode(self, node):
        """
        Getter for all the Constraints implemented at a desired node as a dict.

        Args:
            node: the desired node to be scoped

        Returns:
            a dict of constraint
        """
        if 'n' + str(node) in self.cnstr_impl:
            return self.cnstr_impl['n' + str(node)]

    def getCostFImplAtNode(self, node):
        """
        Getter for all the Cost Functions implemented at a desired node asa a dict.

        Args:
            node: the desired node to be scoped

        Returns:
            a dict of cost functions
        """
        if 'n' + str(node) in self.costfun_impl:
            return self.costfun_impl['n' + str(node)]

    def getCostFImplDict(self):
        """
        Getter for all the Cost Functions implemented at each node as a dict.

        Returns:
            a nested dict of nodes and cost functions {node: cost function}
        """
        return self.costfun_impl

    def getCnstrFImplDict(self):
        """
        Getter for all the Constraint implemented at each node as a dict.

        Returns:
            a nested dict of nodes and constraint functions {node: constraint function}
        """
        return self.cnstr_impl

    def getCostFImplSum(self):
        """
        Getter for the sum of all the cost functions.

        Returns:
            cumulative value of the cost functions at each node
        """
        costfun_impl = list()
        for node in self.costfun_impl.values():
            for elem in node.values():
                costfun_impl.append(elem)

        return cs.sum1(cs.vertcat(*costfun_impl))

    def getCnstrFList(self):
        """
        Getter for all the Constraint Functions implemented at each node as a SX list.

        Returns:
            a SX vector of constraint functions
        """
        cnstr_impl = list()
        for node in self.cnstr_impl.values():
            for elem in node.values():
                cnstr_impl.append(elem['val'])

        return cs.vertcat(*cnstr_impl)

    def getCostFList(self):
        """
        Getter for all the Cost Functions implemented at each node as a SX list.

        Returns:
            a SX vector of cost functions
        """

        cost_impl = list()
        for node in self.costfun_impl.values():
            for elem in node.values():
                cost_impl.append(elem)

        return cs.vertcat(*cost_impl)

    def getLowerBoundsList(self):
        """
        Getter for all the lower bounds of the constraint functions.

        Returns:
            an array containing all the lower bound values. Rows: dimensions. Columns: nodes.
        """
        lbg = np.zeros(self.getCnstrDim())
        # the array is organized as a matrix
        # - rows: dimension
        # - columns: nodes

        j = 0
        for node in self.cnstr_impl.values():
            for elem in node.values():
                lbg[j:j+elem['lb'].shape[0]] = elem['lb']
                j = j+elem['lb'].shape[0]

        return lbg

    def getUpperBoundsList(self):
        """
        Getter for all the lower bounds of the constraint functions.

        Returns:
            an array containing all the upper bound values
        """
        ubg = np.zeros(self.getCnstrDim())
        # the array is organized as a matrix
        # - rows: dimension
        # - columns: nodes

        j = 0
        for node in self.cnstr_impl.values():
            for elem in node.values():
                ubg[j:j + elem['ub'].shape[0]] = elem['ub']
                j = j + elem['ub'].shape[0]

        return ubg

    def setNNodes(self, n_nodes):
        """
        set a desired number of nodes to Function Container.

        Args:
            n_nodes: the desired number of nodes to be set
        """
        # this is required to update the function_container EACH time a new number of node is set
        # todo check!
        self.nodes = n_nodes

        for cnstr in self.cnstr_container.values():
            cnstr.setNodes([i for i in cnstr.getNodes() if i in range(self.nodes)], erasing=True)

        for costfun in self.costfun_container.values():
            costfun.setNodes([i for i in costfun.getNodes() if i in range(self.nodes)], erasing=True)

    def getNCnstrFun(self):
        """
        Getter for the number of constraint functions present in the function container
        Returns:
            number of constraint functions
        """
        return len(self.cnstr_container)

    def getNCostFun(self):
        """
        Getter for the number of cost functions present in the function container
        Returns:
            number of cost functions
        """
        return len(self.costfun_container)

    def clear(self):
        """
        Clears all the implemented function. Both the containers containing the implemented constraint and cost function are cleared.
        """
        self.cnstr_impl.clear()
        self.costfun_impl.clear()

    def serialize(self):
        """
        Serialize the Function Container. Used to save it.

        Returns:
            instance of serialized Function Container

        """
        for name, item in self.cnstr_container.items():
            self.cnstr_container[name] = item.serialize()

        for node, item in self.cnstr_impl.items():
            for name, elem in item.items():
                self.cnstr_impl[node][name]['val'] = elem['val'].serialize()

        for name, item in self.costfun_container.items():
            self.costfun_container[name] = item.serialize()

        for node, item in self.costfun_impl.items():
            for name, elem in item.items():
                self.costfun_impl[node][name] = elem.serialize()

    def deserialize(self):
        """
        Deerialize the Function Container. Used to load it.

        Returns:
            instance of deserialized Function Container

        """
        for name, item in self.cnstr_container.items():
            self.cnstr_container[name] = item.deserialize()

        for node in self.cnstr_impl.keys():
            for name, item in self.cnstr_impl[node].items():
                self.cnstr_impl[node][name]['val'] = cs.SX.deserialize(item['val'])

        # these are CASADI functions
        for name, item in self.costfun_container.items():
            self.costfun_container[name] = item.deserialize()

        # these are SX variables
        for node, item in self.costfun_impl.items():
            for name, elem in item.items():
                self.costfun_impl[node][name] = cs.SX.deserialize(elem)





if __name__ == '__main__':

    x = cs.SX.sym('x', 2)
    y = cs.SX.sym('y', 2)
    fun = x+y
    used_var = dict(x=x, y=y)
    funimpl = Function('dan', fun, used_var, 1)

    funimpl = funimpl.serialize()
    print('===PICKLING===')
    funimpl_serialized = pickle.dumps(funimpl)
    print(funimpl_serialized)
    print('===DEPICKLING===')
    funimpl_new = pickle.loads(funimpl_serialized)


