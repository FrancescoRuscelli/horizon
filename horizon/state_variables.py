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
    def __init__(self, tag, dim):
        super(AbstractVariable, self).__init__(cs.SX.sym(tag, dim))

        self.tag = tag
        self.dim = dim
        self.offset = 0

    def getDim(self):
        return self.dim

class Parameter(AbstractVariable):
    def __init__(self, tag, dim):
        super(Parameter, self).__init__(tag, dim)
        self.value = np.zeros(self.dim)

    def assign(self, vals):
        vals = misc.checkValueEntry(vals)

        if vals.shape[0] != self.dim:
            raise Exception('Wrong dimension of parameter values inserted.')

        self.value = vals

    def getValue(self):
        return self.value

class SingleVariable(AbstractVariable):
    def __init__(self, tag, dim, dummy_nodes):
        super(SingleVariable, self).__init__(tag, dim)

        self.var_impl = dict()
        # todo do i create another var or do I use the SX var inside SingleVariable?
        self.var_impl['var'] = cs.SX.sym(self.tag + '_impl', self.dim)
        self.var_impl['lb'] = np.full(self.dim, -np.inf)
        self.var_impl['ub'] = np.full(self.dim, np.inf)
        self.var_impl['w0'] = np.zeros(self.dim)

    def setLowerBounds(self, bounds):

        bounds = misc.checkValueEntry(bounds)

        if bounds.shape[0] != self.dim:
            raise Exception('Wrong dimension of lower bounds inserted.')

        self.var_impl['lb'] = bounds

    def setUpperBounds(self, bounds):

        bounds = misc.checkValueEntry(bounds)

        if bounds.shape[0] != self.dim:
            raise Exception('Wrong dimension of upper bounds inserted.')

        self.var_impl['ub'] = bounds

    def setBounds(self, lb, ub):
        self.setLowerBounds(lb)
        self.setUpperBounds(ub)

    def setInitialGuess(self, val):

        val = misc.checkValueEntry(val)

        if val.shape[0] != self.dim:
            raise Exception('Wrong dimension of initial guess inserted.')

        self.var_impl['w0'] = val

    def getImpl(self, dummy_node):

        var_impl = self.var_impl['var']
        return var_impl

    def _getVals(self, val_type, dummy_node):
        if dummy_node is None:
            vals = np.array([self.var_impl[val_type]]).T
        else:
            vals = self.var_impl[val_type]

        return vals

    def getLowerBounds(self, dummy_node=None):
        return self._getVals('lb', dummy_node)

    def getUpperBounds(self, dummy_node=None):
        return self._getVals('ub', dummy_node)

    def getBounds(self, dummy_node=None):
        return self.getLowerBounds(dummy_node), self.getUpperBounds(dummy_node)

    def getInitialGuess(self, dummy_node=None):
        return self._getVals('w0', dummy_node)

    def getNodes(self):
        return [-1]

    def getVarOffsetDict(self):
        return dict()

    def getImplDim(self):
        return self.shape[0]

class Variable(AbstractVariable):
    def __init__(self, tag, dim, nodes):
        super(Variable, self).__init__(tag, dim)

        self.nodes = nodes

        # self.var = cs.SX.sym(tag, dim)
        self.var_offset = dict()
        self.var_impl = dict()

        # todo project it as soon as I create the variable. Ok?
        self._project()

    def setLowerBounds(self, bounds, nodes=None):

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
        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def setInitialGuess(self, val, nodes=None):

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
        return self.var_offset

    def _setNNodes(self, n_nodes):

        # todo this is because I must manage Variable, InputVariable, StateVariable in different ways.
        self.nodes = n_nodes
        self._project()

    def _project(self):
        # state_var_impl --> dict
        #  - key: nodes (n0, n1, ...)
        #  - val: dict with name and value of implemented variable
        # old_var_impl = copy.deepcopy(self.var_impl)
        # self.var_impl.clear()
        new_var_impl = dict()

        for n in self.nodes:
            if 'n' + str(n) in self.var_impl:
                new_var_impl['n' + str(n)] = self.var_impl['n' + str(n)]
            else:
                var_impl = cs.SX.sym(self.tag + '_' + str(n), self.dim)
                new_var_impl['n' + str(n)] = dict()
                new_var_impl['n' + str(n)]['var'] = var_impl
                new_var_impl['n' + str(n)]['lb'] = np.full(self.dim, -np.inf)
                new_var_impl['n' + str(n)]['ub'] = np.full(self.dim, np.inf)
                new_var_impl['n' + str(n)]['w0'] = np.zeros(self.dim)

        self.var_impl = new_var_impl

    # todo project only at node n (it is used if I want to reproject at each node)
    # def _projectN(self, n):
    #
    #     # state_var_impl --> dict
    #     #  - key: nodes (n0, n1, ...)
    #     #  - val: dict with name and value of implemented variable
    #     var_impl = cs.SX.sym(self.tag + '_' + str(n), self.dim)
    #     self.var_impl['n' + str(n)] = dict()
    #     self.var_impl['n' + str(n)]['var'] = var_impl
    #     self.var_impl['n' + str(n)]['lb'] = [-np.inf] * self.dim
    #     self.var_impl['n' + str(n)]['ub'] = [np.inf] * self.dim
    #     self.var_impl['n' + str(n)]['w0'] = [0] * self.dim
    #
    #     return var_impl


    def getImpl(self, node):
        # todo this is another option: reproject everytime one asks for .getImpl
        # var_impl = self._projectN(node)
        var_impl = self.var_impl['n' + str(node)]['var']
        return var_impl

    def _getVals(self, val_type, node):
        if node is None:
            vals = np.zeros([self.shape[0], len(self.nodes)])
            for dim in range(self.shape[0]):
                vals[dim, :] = np.hstack([self.var_impl['n' + str(i)][val_type][dim] for i in self.nodes])
        else:
            vals = self.var_impl['n' + str(node)][val_type]
        return vals

    def getLowerBounds(self, node=None):
        return self._getVals('lb', node)

    def getUpperBounds(self, node=None):
        return self._getVals('ub', node)

    def getBounds(self, node=None):
        return self.getLowerBounds(node), self.getUpperBounds(node)

    def getInitialGuess(self, node=None):
        return self._getVals('w0', node)

    def getImplDim(self):
        return self.shape[0] * len(self.getNodes())

    def getNodes(self):
        return self.nodes

    def __reduce__(self):
        return (self.__class__, (self.tag, self.dim, self.nodes, ))


class InputVariable(Variable):
    def __init__(self, tag, dim, nodes):
        super(InputVariable, self).__init__(tag, dim, nodes)

class StateVariable(Variable):
    def __init__(self, tag, dim, nodes):
        super(StateVariable, self).__init__(tag, dim, nodes)

class AbstractAggregate():

    def __init__(self, *args: AbstractVariable):
        self.var_list = [item for item in args]

    def getVars(self) -> cs.SX:
        return cs.vertcat(*self.var_list)

    def __iter__(self):
        yield from self.var_list

    def __getitem__(self, ind):
        return self.var_list[ind]


class Aggregate(AbstractAggregate):

    def __init__(self, *args):
        super().__init__(*args)

    def getVarOffset(self, offset):
        var_list = list()
        for var in self.var_list:
            var_list.append(var.getVarOffset(offset))

        return AbstractAggregate(*var_list)

    def addVariable(self, var):
        self.var_list.append(var)

    def setBounds(self, lb, ub, nodes=None):
        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def setLowerBounds(self, lb, nodes=None):
        idx = 0
        for var in self:
            nv = var.shape[0]
            var.setLowerBounds(lb[idx:idx+nv], nodes)
            idx += nv

    def setUpperBounds(self, ub, nodes=None):
        idx = 0
        for var in self:
            nv = var.shape[0]
            var.setUpperBounds(ub[idx:idx+nv], nodes)
            idx += nv

    def getBounds(self, node):
        lb = self.getLowerBounds(node)
        ub = self.getUpperBounds(node)

        return lb, ub

    def getLowerBounds(self, node):
        return np.hstack([var.getLowerBounds(node) for var in self])

    def getUpperBounds(self, node):
        return np.hstack([var.getUpperBounds(node) for var in self])

class StateAggregate(Aggregate):
    def __init__(self, *args: StateVariable):
        super().__init__(*args)

class InputAggregate(Aggregate):
    def __init__(self, *args: InputVariable):
        super().__init__(*args)


class VariablesContainer:
    def __init__(self, nodes, logger=None):

        self.logger = logger
        self.nodes = nodes

        self.vars = OrderedDict()
        self.vars_impl = OrderedDict()

        self.pars = OrderedDict()

    def createVar(self, var_type, name, dim, active_nodes):

        if active_nodes is not None:
            active_nodes = misc.checkNodes(active_nodes, range(self.nodes))

        var = var_type(name, dim, active_nodes)
        self.vars[name] = var

        if self.logger:
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug('Creating variable {} as {}'.format(name, var_type))

        return var

    def setVar(self, name, dim, active_nodes=None):
        if active_nodes is None:
            var_type = SingleVariable
        else:
            var_type = Variable

        var = self.createVar(var_type, name, dim, active_nodes)
        return var

    def setStateVar(self, name, dim):
        var = self.createVar(StateVariable, name, dim, range(self.nodes))
        return var

    def setInputVar(self, name, dim):
        var = self.createVar(InputVariable, name, dim, range(self.nodes-1))
        return var

    def setSingleVar(self, name, dim):
        var = self.createVar(SingleVariable, name, dim, None)
        return var

    def setParameter(self, name, dim):
        par = Parameter(name, dim)
        self.pars[name] = par

        if self.logger:
            if self.logger.isEnabledFor(logging.DEBUG):
                self.logger.debug(f'Creating parameter "{name}"')

        return par

    def getParameterDict(self):
        return self.pars

    def getParameterList(self):
        return cs.vertcat(*self.pars.values())

    def getParameterValues(self):
        par_list = [par.getValue() for par in self.pars.values()]
        return cs.vertcat(*par_list)

    def getVarsDim(self):
        var_dim_tot = 0
        for var in self.vars.values():
            var_dim_tot += var.getImplDim()
        return var_dim_tot

    def getStateVars(self):
        state_vars = dict()
        for name, var in self.vars.items():
            if isinstance(var, StateVariable):
                state_vars[name] = var

        return state_vars

    def getInputVars(self):
        input_vars = dict()
        for name, var in self.vars.items():
            if isinstance(var, InputVariable):
                input_vars[name] = var

        return input_vars

    def getVarAbstrDict(self, past=True):

        if past:
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

    def getVarImpl(self, name, k):


        if isinstance(self.vars[name], SingleVariable):
            k = None

        node_name = 'n' + str(k)

        if node_name in self.vars_impl:
            var = self.vars_impl[node_name][name]['var']
        else:
            var = None

        return var

    def getVarImplAtNode(self, k):
        if 'n' + str(k) in self.vars_impl:
            return self.vars_impl['n' + str(k)]
        else:
            return None

    def getVarImplDict(self):
        return self.vars_impl

    def getVarImplList(self):
        '''
        return: SX vector (vertcat) of all the implemented variables at each node
            used by problem.py to retrieve the final vector of optimization variables
        '''

        state_var_impl_list = list()
        for val in self.vars_impl.values():
            for var_abstract in val.keys():
                # get from state_var_impl the relative var

                state_var_impl_list.append(val[var_abstract]['var'])

        return cs.vertcat(*state_var_impl_list)

    def _getVarInfoList(self, bound_type):

        state_var_bound_list = np.zeros(self.getVarsDim())

        j = 0
        for node, val in self.vars_impl.items():
            for var_abstract in val.keys():
                var = val[var_abstract][bound_type]
                dim = val[var_abstract][bound_type].shape[0]
                state_var_bound_list[j:j + dim] = var
                j = j + dim

        return state_var_bound_list

    def getBoundsMinList(self):
        return self._getVarInfoList('lb')

    def getBoundsMaxList(self):
        return self._getVarInfoList('ub')

    def getInitialGuessList(self):
        return self._getVarInfoList('w0')

    def _fillVar(self, name, node, val):
        # todo bounds are not necessary here
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


    def build(self):
        '''
        fill the dictionary "state_var_impl"
           - key: nodes (nNone, n0, n1, ...) nNone contains single variables that are not projected in nodes
           - val: dict with name and value of implemented variable
        '''

        # contains single vars that do not need to be projected
        self.vars_impl['nNone'] = dict()

        for node in range(self.nodes):
            # contains vars that are projected along the nodes
            self.vars_impl['n' + str(node)] = dict()

            for name, val in self.vars.items():

                if isinstance(val, SingleVariable):
                    self._fillVar(name, None, val)
                    continue

                if node in self.vars[name].getNodes():
                    self._fillVar(name, node, val)

    def updateBounds(self):

        for node in self.vars_impl.keys():
            for name, state_var in self.vars_impl[node].items():
                k = node[node.index('n') + 1:]
                state_var['lb'] = self.vars[name].getLowerBounds(k)
                state_var['ub'] = self.vars[name].getUpperBounds(k)

    def updateInitialGuess(self):

        for node in self.vars_impl.keys():
            for name, state_var in self.vars_impl[node].items():
                k = node[node.index('n') + 1:]
                state_var['w0'] = self.vars[name].getInitialGuess(k)

    def setNNodes(self, n_nodes):

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



    def clear(self):
        self.vars_impl.clear()


    def serialize(self):

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
