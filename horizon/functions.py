import casadi as cs
import numpy as np
from horizon import misc_function as misc
from collections import OrderedDict
import pickle
import time
from typing import Union, Iterable
# from horizon.type_doc import BoundsDict

class Function:

    """
        Function of Horizon: generic function of SX values from CASADI.

        Notes:
            This function is an abstract representation of its projection over the nodes of the optimization problem.
            An abstract function gets internally implemented at each node, using the variables at that node.
    """
    def __init__(self, name: str, f: cs.SX, used_vars: list, used_pars: list, nodes: Union[int, Iterable]):
        """
        Initialize the Horizon Function.

        Args:
            name: name of the function
            f: SX function
            used_vars: variable used in the function
            used_pars: parameters used in the function
            nodes: nodes the function is active on
        """

        self._f = f
        self._name = name
        self._nodes = []

        # todo isn't there another way to get the variables from the function g?
        self.vars = used_vars
        self.pars = used_pars

        # create function of CASADI, dependent on (in order) [all_vars, all_pars]
        self._fun = cs.Function(name, self.vars + self.pars, [self._f])
        self._fun_impl = dict()

        self.setNodes(nodes)

    def getName(self) -> str:
        """
        Getter for the name of the function

        Returns:
            name of the function
        """
        return self._name

    def getDim(self) -> int:
        """
        Getter for the dimension of the function

        Returns:
            dimension of the function
        """
        return self._f.shape[0]

    def getFunction(self) -> cs.Function:
        """
        Getter for the CASADI function

        Returns:
            instance of the CASADI function
        """
        return self._fun

    def getImpl(self, nodes = None):
        """
        Getter for the CASADI function implemented at the desired node
        Args:
            node: the desired node of the implemented function to retrieve

        Returns:
            instance of the CASADI function at the desired node
        """
        if nodes is None:
            nodes = self._nodes

        nodes = misc.checkNodes(nodes, self._nodes)
        fun_impl = cs.vertcat(*[self._fun_impl['n' + str(i)] for i in nodes])

        return fun_impl

    def _project(self, used_vars):
        """
        Implements the function at the desired node using the desired variables.

        Args:
            used_vars: the variable used at the desired node

        Returns:
            the implemented function
        """
        fun_eval = self._fun(*used_vars)

        for i in range(len(self._nodes)):
            self._fun_impl['n' + str(self._nodes[i])] = fun_eval[:, i]

        # reshape it as a vector for solver
        # fun_eval_vector = cs.reshape(fun_eval, (self.getDim() * len(self.getNodes()), 1))


    def getNodes(self) -> list:
        """
        Getter for the active nodes of the function.

        Returns:
            a list of the nodes where the function is active

        """
        return self._nodes

    def setNodes(self, nodes, erasing=False):
        """
        Setter for the active nodes of the function.

        Args:
            nodes: list of desired active nodes.
            erasing: choose if the inserted nodes overrides the previous active nodes of the function. 'False' if not specified.
        """
        if erasing:
            self._nodes.clear()

        # adding active nodes to function nodes
        for i in nodes:
            if i not in self._nodes:
                self._nodes.append(i)
                self._nodes.sort()

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
        self._f = self._f.serialize()

        for name, data in self.vars.items():
            self.vars[name] = data.serialize()

        for node, item in self._fun_impl.items():
            self._fun_impl[node] = item.serialize()

        self._fun = self.fun.serialize()

        return self

    def deserialize(self):
        """
        Deserialize the Function. Used to load it.

        Returns:
            deserialized instance of Function
        """
        self._f = cs.SX.deserialize(self._f)

        for name, data in self.vars.items():
            self.vars[name] = cs.SX.deserialize(data)

        for node, item in self._fun_impl.items():
            self._fun_impl[node] = cs.Function.deserialize(item)

        self._fun = cs.Function.deserialize(self._fun)

        return self


class Constraint(Function):
    """
    Constraint Function of Horizon.
    """
    def __init__(self, name: str, f: cs.SX, used_vars: list, used_pars: list, nodes: Union[int, Iterable], bounds =None):
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

            if 'lb' in bounds:
                bounds['lb'] = misc.checkValueEntry(bounds['lb'])
                if bounds['lb'].shape[0] != self.getDim():
                    raise Exception('Wrong dimension of lower bounds inserted.')
                if 'ub' not in bounds:
                    bounds['ub'] = np.full(f.shape[0], np.inf)

            if 'ub' in bounds:
                bounds['ub'] = misc.checkValueEntry(bounds['ub'])
                if bounds['ub'].shape[0] != self.getDim():
                    raise Exception('Wrong dimension of upper bounds inserted.')
                if 'lb' not in bounds:
                    bounds['lb'] = np.full(f.shape[0], -np.inf)

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
            nodes = self._nodes
        else:
            nodes = misc.checkNodes(nodes, self._nodes)

        bounds = misc.checkValueEntry(bounds)

        # todo what if bounds does not have shape!
        if bounds.shape[0] != self.getDim():
            raise Exception('Wrong dimension of lower bounds inserted.')

        for node in nodes:
            if node in self._nodes:
                self.bounds['n' + str(node)].update({'lb': bounds})

    def setUpperBounds(self, bounds, nodes=None):
        """
        Setter for the upper bounds of the function.

        Args:
            bounds: desired bounds of the function
            nodes: nodes of the function the bounds are applied on. If not specified, the function is bounded along ALL the nodes.
        """
        if nodes is None:
            nodes = self._nodes
        else:
            nodes = misc.checkNodes(nodes, self._nodes)

        bounds = misc.checkValueEntry(bounds)

        if bounds.shape[0] != self.getDim():
            raise Exception('Wrong dimension of upper bounds inserted.')

        for node in nodes:
            if node in self._nodes:
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

    def _getVals(self, val_type: str, nodes=None):
        """
        wrapper function to get the desired argument from the constraint.

        Args:
            val_type: type of the argument to retrieve
            node: desired node at which the argument is retrieved. If not specified, this returns the desired argument at all nodes.

        Returns:
            value/s of the desired argument
        """
        if nodes is None:
            nodes = self._nodes

        nodes = misc.checkNodes(nodes, self._nodes)

        vals = np.hstack([self.bounds['n' + str(i)][val_type] for i in nodes])

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
            self._nodes.clear()

        # adding to function nodes
        for i in nodes:
            if i not in self._nodes:
                self._nodes.append(i)
                self._nodes.sort()
                # for all the "new nodes" that weren't there, add default bounds
                if 'n' + str(i) not in self.bounds:
                    self.bounds['n' + str(i)] = dict(lb=np.full(self._f.shape[0], 0.),
                                                     ub=np.full(self._f.shape[0], 0.))

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
    def __init__(self, logger=None):
        """
        Initialize the Function Container.

        Args:
            logger: a logger reference to log data
        """
        self._logger = logger

        # containers for the constraints
        self._cnstr_container = OrderedDict()

        # containers for the cost function
        self._costfun_container = OrderedDict()

    def addFunction(self, fun: Function):
        """
        Add a function to the Function Container.

        Args:
            fun: a Function (can be Constraint or Cost Function) o add
        """
        if fun.getType() == 'constraint':
            if fun.getName() not in self._cnstr_container:
                self._cnstr_container[fun.getName()] = fun
            else:
                raise Exception(f'Function name "{fun.getName()}" already inserted.')
        elif fun.getType() == 'costfunction':
            if fun.getName() not in self._costfun_container:
                self._costfun_container[fun.getName()] = fun
            else:
                raise Exception(f'Function name "{fun.getName()}" already inserted.')
        elif fun.getType() == 'generic':
            print('functions.py: generic not implemented')

    def removeFunction(self, fun_name: str):
        """
        Remove a function from the Function Container.

        Args:
            fun_name: name of the function to be removed
        """
        if fun_name in self._cnstr_container:
            del self._cnstr_container[fun_name]
            return True
        elif fun_name in self._costfun_container:
            del self._costfun_container[fun_name]
            return True
        else:
            return False

    def getFunction(self, fun_name: str):
        """
        Getter for a Function inside the Function Container.
        Args:
            fun_name: name of the function to retrieve
        """
        if fun_name in self._cnstr_container:
            return self._cnstr_container[fun_name]
        elif fun_name in self._costfun_container:
            return self._costfun_container[fun_name]
        else:
            return None

    def getCnstr(self, name=None) -> OrderedDict:
        """
        Getter for the dictionary of all the abstract constraint functions.

        Args:
            name of constraint. If not specified, returns all of them
        Returns:
            ordered dict of the functions {name: fun}
        """
        if name is None:
            cnsrt_dict = self._cnstr_container
        else:
            cnsrt_dict = self._cnstr_container[name]
        return cnsrt_dict

    def getCost(self, name=None) -> OrderedDict:
        """
        Getter for the dictionary of all the abstract cost functions.

        Args:
            name of constraint. If not specified, returns all of them

        Returns:
            ordered dict of the functions {name: fun}
        """
        if name is None:
            cost_dict = self._costfun_container
        else:
            cost_dict = self._costfun_container[name]
        return cost_dict

    def getCnstrDim(self) -> int:
        """
        Getter for the dimension of all the constraints
        Returns:
            the total dimension of the constraint
        """
        total_dim = 0
        for cnstr in self._cnstr_container.values():
            total_dim += cnstr.getDim() * len(cnstr.getNodes())

        return total_dim

    def setNNodes(self, n_nodes):
        """
        set a desired number of nodes to Function Container.
        Args:
            n_nodes: the desired number of nodes to be set
        """
        # this is required to update the function_container EACH time a new number of node is set
        for cnstr in self._cnstr_container.values():
            cnstr.setNodes([i for i in cnstr.getNodes() if i in range(n_nodes)], erasing=True)

        for costfun in self._costfun_container.values():
            costfun.setNodes([i for i in costfun.getNodes() if i in range(n_nodes)], erasing=True)

    def serialize(self):
        """
        Serialize the Function Container. Used to save it.

        Returns:
            instance of serialized Function Container

        """
        raise Exception('serialize yet to be re-implemented')
        for name, item in self._cnstr_container.items():
            self._cnstr_container[name] = item.serialize()

        for name, item in self._costfun_container.items():
            self._costfun_container[name] = item.serialize()



    def deserialize(self):
        """
        Deerialize the Function Container. Used to load it.

        Returns:
            instance of deserialized Function Container

        """
        raise Exception('deserialize yet to be re-implemented')
        for name, item in self._cnstr_container.items():
            self._cnstr_container[name] = item.deserialize()

        # these are CASADI functions
        for name, item in self._costfun_container.items():
            self._costfun_container[name] = item.deserialize()




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


