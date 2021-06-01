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

    def setLowerBounds(self, node, bounds):
        if isinstance(node, list):
            for n in range(node[0], node[1]):
                self.var_impl['n' + str(n)]['lb'] = bounds
        else:
            self.var_impl['n' + str(node)]['lb'] = bounds
        
    def setUpperBounds(self, node, bounds):
        if isinstance(node, list):
            for n in range(node[0], node[1]):
                self.var_impl['n' + str(n)]['ub'] = bounds
        else:
            self.var_impl['n' + str(node)]['ub'] = bounds

    def setBounds(self, node, lb, ub):
        self.setLowerBounds(node, lb)
        self.setUpperBounds(node, ub)

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


class StateVariables:

    def __init__(self, nodes):

        self.nodes = nodes

        self.state_var = OrderedDict()
        self.state_var_prev = OrderedDict()

        self.state_var_impl = OrderedDict()

    def setVar(self, name, dim, prev_nodes=None):

        # todo what if 'prev_nodes' it is a list
        createTag = lambda name, node: name + str(node) if node is not None else name
        checkExistence = lambda name, node: True if prev_nodes is None else True if name in self.state_var else False
        tag = createTag(name, prev_nodes)

        if checkExistence(name, prev_nodes):
            var = StateVariable(tag, dim, self.nodes)
            if prev_nodes is None:
                self.state_var[tag] = var
            else:
                self.state_var_prev[tag] = var
            return var
        else:
            raise Exception('Yet to declare the present variable!')


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
            for var_abstract in self.state_var:
                # get from state_var_impl the relative var
                # todo right now, if a variable in state_var_impl is NOT in state_var, it won't be considered in state_var_impl_list
                state_var_impl_list.append(val[var_abstract]['var'])

        return cs.vertcat(*state_var_impl_list)

    def getBoundsMinList(self):

        state_var_bound_list = list()

        for node, val in self.state_var_impl.items():
            for var_abstract in self.state_var:
                # get from state_var_impl the relative var
                # todo right now, if a variable in state_var_impl is NOT in state_var, it won't be considered in state_var_impl_list
                state_var_bound_list.append(val[var_abstract]['lb'])

        return cs.vertcat(*state_var_bound_list)

    def getBoundsMaxList(self):

        state_var_bound_list = list()

        for node, val in self.state_var_impl.items():
            for var_abstract in self.state_var:
                # get from state_var_impl the relative var
                # todo right now, if a variable in state_var_impl is NOT in state_var, it won't be considered in state_var_impl_list
                state_var_bound_list.append(val[var_abstract]['ub'])

        return cs.vertcat(*state_var_bound_list)

    def getVarAbstrDict(self):
        # this is used to check the variable existing in the function. It requires all the variables and the previous variables
        var_abstr_dict = {**self.state_var, **self.state_var_prev}
        return var_abstr_dict

    def update(self, k):
        # state_var_impl --> dict
        #  - key: nodes (n0, n1, ...)
        #  - val: dict with name and value of implemented variable

        self.state_var_impl['n' + str(k)] = dict()
        # implementation of current state variable
        for name, val in self.state_var.items():
            var_impl = self.state_var[name].getImpl(k)
            var_bound_min = self.state_var[name].getBoundMin(k)
            var_bound_max = self.state_var[name].getBoundMax(k)
            var_dict = dict(var=var_impl, lb=var_bound_min, ub=var_bound_max)
            self.state_var_impl['n' + str(k)].update({name : var_dict})

    def updateBounds(self):

        for node in self.state_var_impl.keys():
            for name, state_var in self.state_var_impl[node].items():
                k = node[node.index('n') + 1:]
                state_var['lb'] = self.state_var[name].getBoundMin(k)
                state_var['ub'] = self.state_var[name].getBoundMax(k)
            # self.state_var_impl


if __name__ == '__main__':


    sv = StateVariables(3)
    x = sv.setVar('x', 2)
    # sv.setVar('y', 2)
    x_prev = sv.setVar('x', 2, -2)

    # print(x.bounds)
    # print(x_prev)

    for i in range(4):
        sv.update(i)

    print('state_var_prev', sv.state_var_prev)
    print('state_var_impl', sv.state_var_impl)

    print('sv.getVarAbstrDict()', sv.getVarAbstrDict())
    print('sv.getVarAbstrList()', sv.getVarAbstrList())
    print('sv.getVarImplList()', sv.getVarImplList())
    # print('sv.getVarImpl()', sv.getVarImpl('x-2', 0))


