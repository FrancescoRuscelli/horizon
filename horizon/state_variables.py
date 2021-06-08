import pprint

import casadi as cs
from collections import OrderedDict

import numpy as np
import types

class StateVariable(cs.SX):
    def __init__(self, tag, dim, nodes):
        super(StateVariable, self).__init__(cs.SX.sym(tag, dim))

        self.tag = tag
        self.dim = dim
        self.nodes = nodes

        # self.var = cs.SX.sym(tag, dim)
        self.var_impl = dict()

        self._project()

    def setLowerBounds(self, bounds, nodes=None):

        if nodes is None:
            nodes = [0, self.nodes]

        if isinstance(nodes, list):
            for n in range(nodes[0], nodes[1]):
                self.var_impl['n' + str(n)]['lb'] = bounds
        else:
            self.var_impl['n' + str(nodes)]['lb'] = bounds
        
    def setUpperBounds(self, bounds, nodes=None):

        if nodes is None:
            nodes = [0, self.nodes]

        if isinstance(nodes, list):
            for n in range(nodes[0], nodes[1]):
                self.var_impl['n' + str(n)]['ub'] = bounds
        else:
            self.var_impl['n' + str(nodes)]['ub'] = bounds

    def setBounds(self, lb, ub, nodes=None):
        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def setInitialGuess(self, val, nodes=None):

        if nodes is None:
            nodes = [0, self.nodes]

        if isinstance(nodes, list):
            for n in range(nodes[0], nodes[1]):
                self.var_impl['n' + str(n)]['w0'] = val
        else:
            self.var_impl['n' + str(nodes)]['w0'] = val

    def _project(self):
        # state_var_impl --> dict
        #  - key: nodes (n0, n1, ...)
        #  - val: dict with name and value of implemented variable
        for n in range(self.nodes):
            var_impl = cs.SX.sym(self.tag + '_' + str(n), self.dim)
            self.var_impl['n' + str(n)] = dict()
            self.var_impl['n' + str(n)]['var'] = var_impl
            self.var_impl['n' + str(n)]['lb'] = [-np.inf] * self.dim
            self.var_impl['n' + str(n)]['ub'] = [np.inf] * self.dim
            self.var_impl['n' + str(n)]['w0'] = [0] * self.dim

    def getImpl(self, node):
        var_impl = self.var_impl['n' + str(node)]['var']
        return var_impl

    def getBoundMin(self, node):
        bound_min = self.var_impl['n' + str(node)]['lb']
        return bound_min

    def getBoundMax(self, node):
        bound_max = self.var_impl['n' + str(node)]['ub']
        return bound_max

    def getBounds(self, node):
        return [self.getBoundMin(node), self.getBoundMax(node)]

    def getInitialGuess(self, node):
        initial_guess = self.var_impl['n' + str(node)]['w0']
        return initial_guess


class InputVariable(StateVariable):
    def __init__(self, tag, dim, nodes):
        super(InputVariable, self).__init__(tag, dim, nodes)

    def _project(self):
        # state_var_impl --> dict
        #  - key: nodes (n0, n1, ...)
        #  - val: dict with name and value of implemented variable
        for n in range(self.nodes-1):
            var_impl = cs.SX.sym(self.tag + '_' + str(n), self.dim)
            self.var_impl['n' + str(n)] = dict()
            self.var_impl['n' + str(n)]['var'] = var_impl
            self.var_impl['n' + str(n)]['lb'] = [-np.inf] * self.dim
            self.var_impl['n' + str(n)]['ub'] = [np.inf] * self.dim
            self.var_impl['n' + str(n)]['w0'] = [0] * self.dim

class StateVariables:

    def __init__(self, nodes, logger=None):

        self.logger = logger
        self.nodes = nodes

        self.state_var = OrderedDict()
        self.state_var_prev = OrderedDict()

        self.state_var_impl = OrderedDict()

    def setVar(self, var_type, name, dim, prev_nodes):

        # todo what if 'prev_nodes' it is a list
        createTag = lambda name, node: name + str(node) if node is not None else name
        checkExistence = lambda name, node: True if prev_nodes is None else True if name in self.state_var else False
        tag = createTag(name, prev_nodes)

        if self.logger:
            self.logger.debug('Setting variable {} with tag {} as {}'.format(name, tag, var_type))

        if checkExistence(name, prev_nodes):
            var = var_type(tag, dim, self.nodes)
            if prev_nodes is None:
                self.state_var[tag] = var
            else:
                self.state_var_prev[tag] = var
            return var
        else:
            raise Exception('Yet to declare the present variable!')

    def setStateVar(self, name, dim, prev_nodes=None):
        var = self.setVar(StateVariable, name, dim, prev_nodes)
        return var

    def setInputVar(self, name, dim, prev_nodes=None):
        var = self.setVar(InputVariable, name, dim, prev_nodes)
        return var

    def getVarImpl(self, name, k):
        node_name = 'n' + str(k)

        if name.find('-') == -1:
            var = self.state_var_impl[node_name][name]['var']
        else:
            var_name = name[:name.index('-')]
            k_prev = int(name[name.index('-'):])
            node_prev = 'n' + str(k+k_prev)
            var = self.state_var_impl[node_prev][var_name]['var']

        return var

    def getVarImplDict(self):
        return self.state_var_impl

    def getVarImplList(self):

        state_var_impl_list = list()

        for node, val in self.state_var_impl.items():
            for var_abstract in val.keys():
                # get from state_var_impl the relative var

                # todo right now, if a variable in state_var_impl is NOT in state_var, it won't be considered in state_var_impl_list

                state_var_impl_list.append(val[var_abstract]['var'])

        return cs.vertcat(*state_var_impl_list)

    def getBoundsMinList(self):

        state_var_bound_list = list()

        for node, val in self.state_var_impl.items():
            for var_abstract in val.keys():

                state_var_bound_list += val[var_abstract]['lb']

        return state_var_bound_list

    def getBoundsMaxList(self):

        state_var_bound_list = list()

        for node, val in self.state_var_impl.items():
            for var_abstract in val.keys():
                # get from state_var_impl the relative var
                # todo right now, if a variable in state_var_impl is NOT in state_var, it won't be considered in state_var_impl_list
                state_var_bound_list += val[var_abstract]['ub']

        return state_var_bound_list

    def getVarAbstrDict(self, past=True):
        # this is used to check the variable existing in the function. It requires all the variables and the previous variables
        if past:
            var_abstr_dict = {**self.state_var, **self.state_var_prev}
        else:
            var_abstr_dict = self.state_var

        return var_abstr_dict

    def getInitialGuessList(self):

        initial_guess_list = list()

        for node, val in self.state_var_impl.items():
            for var_abstract in val.keys():
                # get from state_var_impl the relative var
                # todo right now, if a variable in state_var_impl is NOT in state_var, it won't be considered in state_var_impl_list
                initial_guess_list += val[var_abstract]['w0']

        return initial_guess_list

    def update(self, k):
        # state_var_impl --> dict
        #  - key: nodes (n0, n1, ...)
        #  - val: dict with name and value of implemented variable

        self.state_var_impl['n' + str(k)] = dict()
        # implementation of current state variable
        for name, val in self.state_var.items():
            if isinstance(val, InputVariable) and k == self.nodes-1:
                continue

            var_impl = self.state_var[name].getImpl(k)

            if self.logger:
                self.logger.debug('Implemented {} of type {}: {}'.format(name, type(val), var_impl))

            # todo this is not necessary right?
            var_bound_min = self.state_var[name].getBoundMin(k)
            var_bound_max = self.state_var[name].getBoundMax(k)
            initial_guess = self.state_var[name].getInitialGuess(k)
            var_dict = dict(var=var_impl, lb=var_bound_min, ub=var_bound_max, w0=initial_guess)
            self.state_var_impl['n' + str(k)].update({name : var_dict})

    def updateBounds(self):

        for node in self.state_var_impl.keys():
            for name, state_var in self.state_var_impl[node].items():
                k = node[node.index('n') + 1:]
                state_var['lb'] = self.state_var[name].getBoundMin(k)
                state_var['ub'] = self.state_var[name].getBoundMax(k)
            # self.state_var_impl

    def updateInitialGuess(self):

        for node in self.state_var_impl.keys():
            for name, state_var in self.state_var_impl[node].items():
                k = node[node.index('n') + 1:]
                state_var['w0'] = self.state_var[name].getInitialGuess(k)


if __name__ == '__main__':


    sv = StateVariables(3)
    x = sv.setVar('x', 2)
    # sv.setVar('y', 2)
    x_prev = sv.setVar('x', 2, -2)

    for i in range(4):
        sv.update(i)

    print('state_var_prev', sv.state_var_prev)
    print('state_var_impl', sv.state_var_impl)

    print('sv.getVarAbstrDict()', sv.getVarAbstrDict())
    print('sv.getVarAbstrList()', sv.getVarAbstrList())
    print('sv.getVarImplList()', sv.getVarImplList())
    # print('sv.getVarImpl()', sv.getVarImpl('x-2', 0))


