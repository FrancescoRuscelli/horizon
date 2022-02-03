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
        all_input = self.vars + self.pars
        all_names = [i.getName() for i in all_input]
        self._fun = cs.Function(name, self.vars + self.pars, [self._f], 
            all_names, ['f'])
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


    def _getUsedVarImpl(self):
        # todo throw with a meaningful error when nodes inserted are wrong
        used_var_impl = list()
        for var in self.vars:
            var_impl = var.getImpl(self.getNodes())
            var_dim = var.getDim()
            # reshape them for all-in-one evaluation of function
            # this is getting all the generic variable x, even if the function has a slice of it x[0:2].
            # It will work. The casadi function takes from x only the right slices afterwards.
            # print(var_impl)
            var_impl_matrix = cs.reshape(var_impl, (var_dim, len(self.getNodes())))
            # generic input --> row: dim // column: nodes
            # [[x_0_0, x_1_0, ... x_N_0],
            #  [x_0_1, x_1_1, ... x_N_1]]
            used_var_impl.append(var_impl_matrix)

        return used_var_impl

    def _getUsedParImpl(self):

        used_par_impl = list()
        for par in self.pars:
            par_impl = par.getImpl(self.getNodes())
            par_dim = par.getDim()
            par_impl_matrix = cs.reshape(par_impl, (par_dim, len(self.getNodes())))
            used_par_impl.append(par_impl_matrix)

        return used_par_impl

    def _project(self):
        """
        Implements the function at the desired node using the desired variables.

        Args:
            used_vars: the variable used at the desired node

        Returns:
            the implemented function
        """
        used_var_impl = self._getUsedVarImpl()
        used_par_impl = self._getUsedParImpl()
        all_vars = used_var_impl+used_par_impl
        fun_eval = self._fun(*all_vars)

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

        # todo this is redundant. If the implemented variables do not change, this is not required, right?
        #   How do I understand when the var impl changed?
        # usually the number of nodes stays the same, while the active nodes of a function may change.
        # If the number of nodes changes, also the variables change. That is when this reprojection is required.
        self._project()

    def getVariables(self, offset=True) -> list:
        """
        Getter for the variables used in the function.

        Returns:
            a dictionary of all the used variable. {name: cs.SX}
        """

        if offset:
            ret = self.vars
        else:
            ret = list()
            for var in self.vars:
                if var.getOffset() == 0:
                    ret.append(var)


        return ret

    def getParameters(self) -> list:
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

    # def __reduce__(self):
    #     """
    #     Experimental function to serialize this element.
    #
    #     Returns:
    #         instance of this element serialized
    #     """
    #     return (self.__class__, (self._name, self._f, self.vars, self.pars, self._nodes, ))

    def serialize(self):
        """
        Serialize the Function. Used to save it.

        Returns:
            serialized instance of Function
        """

        self._f = self._f.serialize()

        for i in range(len(self.vars)):
            self.vars[i] = self.vars[i].serialize()

        for node, item in self._fun_impl.items():
            self._fun_impl[node] = item.serialize()

        # self._fun = self._fun.serialize()

        return self

    def deserialize(self):
        """
        Deserialize the Function. Used to load it.

        Returns:
            deserialized instance of Function
        """

        self._f = cs.SX.deserialize(self._f)

        for i in range(len(self.vars)):
            self.vars[i] = cs.SX.deserialize(self.vars[i])

        for node, item in self._fun_impl.items():
            self._fun_impl[node] = cs.SX.deserialize(item)

        # self._fun = cs.Function.deserialize(self._fun)

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

        if len(nodes) == 0:
            return np.zeros((self.getDim(), 0))

        vals = np.hstack([np.atleast_2d(self.bounds['n' + str(i)][val_type]).T for i in nodes])

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

        self._project()

class CostFunction(Function):
    """
    Cost Function of Horizon.
    """
    def __init__(self, name, f, used_vars, used_pars, nodes):
        """
        Initialize the Cost Function.

        Args:
            name: name of the function
            f: SX function
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

class ResidualFunction(Function):
    """
    Residual Function of Horizon.
    """
    def __init__(self, name, f, used_vars, used_pars, nodes):
        """
        Initialize the Residual Function.

        Args:
            name: name of the function
            f: SX function
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
        return 'residualfunction'

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

        # containers for the constraint functions
        self._cnstr_container = OrderedDict()

        # containers for the cost functions
        self._cost_container = OrderedDict()

    def addFunction(self, fun: Function):
        """
        Add a function to the Function Container.

        Args:
            fun: a Function (can be Constraint or Cost Function) o add
        """
        # todo refactor this using types
        if fun.getType() == 'constraint':
            if fun.getName() not in self._cnstr_container:
                self._cnstr_container[fun.getName()] = fun
            else:
                raise Exception(f'Function name "{fun.getName()}" already inserted.')
        elif fun.getType() == 'costfunction':
            if fun.getName() not in self._cost_container:
                self._cost_container[fun.getName()] = fun
            else:
                raise Exception(f'Function name "{fun.getName()}" already inserted.')
        elif fun.getType() == 'residualfunction':
            if fun.getName() not in self._cost_container:
                self._cost_container[fun.getName()] = fun
            else:
                raise Exception(f'Function name "{fun.getName()}" already inserted.')
        elif fun.getType() == 'generic':
            print('functions.py: generic not implemented')
        else:
            raise Exception('Function type not implemented')

    def removeFunction(self, fun_name: str):
        """
        Remove a function from the Function Container.

        Args:
            fun_name: name of the function to be removed
        """
        if fun_name in self._cnstr_container:
            del self._cnstr_container[fun_name]
            return True
        elif fun_name in self._cost_container:
            del self._cost_container[fun_name]
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
        elif fun_name in self._cost_container:
            return self._cost_container[fun_name]
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
            if name in self._cnstr_container:
                cnsrt_dict = self._cnstr_container[name]
            else:
                cnsrt_dict = None
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
            cost_dict = self._cost_container
        else:
            cost_dict = self._cost_container[name]
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

        # todo should find a way to understand which is the transcription function and project it over all the nodes
        # this is required to update the function_container EACH time a new number of node is set
        for cnstr in self._cnstr_container.values():
            # this is required to update the nodes consistently.
            # For instance, the horizon problem is specified on [0, 1, 2, 3, 4].
            # Consider a function containing an input variable. it is active on [0, 1, 2, 3].
            # I change the nodes to [0, 1, 2, 3]. The function must be updated accordingly: [0, 1, 2]
            # I change the nodes to [0, 1, 2, 3, 4, 5]. The function must be updated accordingly: [0, 1, 2, 4]
            available_nodes = set(range(n_nodes))
            # get only the variable it depends (not all the offsetted variables)
            for var in cnstr.getVariables(offset=False):
                if not var.getNodes() == [-1]:  # todo very bad hack to check if the variable is a SingleVariable (i know it returns [-1]
                    available_nodes.intersection_update(var.getNodes())

            cnstr.setNodes([i for i in cnstr.getNodes() if i in available_nodes], erasing=True)

        for cost in self._cost_container.values():
            available_nodes = set(range(n_nodes))
            for var in cost.getVariables(offset=False):
                if not var.getNodes() == [-1]:
                    available_nodes.intersection_update(var.getNodes())
            cost.setNodes([i for i in cost.getNodes() if i in range(n_nodes)], erasing=True)

    def serialize(self):
        """
        Serialize the Function Container. Used to save it.

        Returns:
            instance of serialized Function Container

        """
        raise Exception('serialize yet to implement')
        for name, item in self._cnstr_container.items():
            self._cnstr_container[name] = item.serialize()


        for name, item in self._cost_container.items():
            self._cost_container[name] = item.serialize()



    def deserialize(self):
        """
        Deerialize the Function Container. Used to load it.

        Returns:
            instance of deserialized Function Container

        """
        raise Exception('serialize yet to implement')
        for name, item in self._cnstr_container.items():
            item.deserialize()
            new_vars = item.getVariables()
            for var in new_vars:
                print(var.getName(), var.getOffset())
                print(f'{item._f} depends on {var}?', cs.depends_on(item._f, var))

        exit()


        for name, item in self._cnstr_container.items():
            self._cnstr_container[name] = item.deserialize()


        # these are CASADI functions
        for name, item in self._cost_container.items():
            self._cost_container[name] = item.deserialize()




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


