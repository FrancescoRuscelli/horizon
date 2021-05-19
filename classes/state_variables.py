import casadi as cs
from collections import OrderedDict

class StateVariables:

    def __init__(self):

        self.state_var = OrderedDict()
        self.state_var_prev = OrderedDict()

        self.state_var_impl = OrderedDict()

    def setVar(self, name, dim, prev_nodes=None):

        # todo what if 'prev_nodes' it is a list
        createTag = lambda name, node: name + str(node) if node is not None else name
        checkExistence = lambda name, node: True if prev_nodes is None else True if name in self.state_var else False
        tag = createTag(name, prev_nodes)

        if checkExistence(name, prev_nodes):
            var = self.var = cs.SX.sym(tag, dim)
            if prev_nodes is None:
                self.state_var[tag] = var
            else:
                self.state_var_prev[tag] = var
            return var
        else:
            raise Exception('Yet to declare the present variable!')

    def getVarImpl(self, names, k):
        # todo isn't it better to do StateVariables[k][name]?
        node_name = 'n' + str(k)
        if isinstance(names, list):
            var_list = list()
            for name in names:
                var_list.append(self.state_var_impl[node_name][name])
            return var_list
        else:
            return self.state_var_impl[node_name][names]

    def getVarImplList(self):

        state_var_impl_list = list()

        for node, val in self.state_var_impl.items():
            for var_abstract in self.state_var:
                # get from state_var_impl the relative var
                # todo right now, if a variable in state_var_impl is NOT in state_var, it won't be considered in state_var_impl_list
                state_var_impl_list.append(val[var_abstract])

        return cs.vertcat(*state_var_impl_list)

    def getVarAbstrDict(self):

        return self.state_var

    def getVarAbstrList(self):

        state_var_abstr_list = self.state_var.values()
        return state_var_abstr_list

    def update(self, k):
        # state_var_impl --> dict
        #  - key: nodes (n0, n1, ...)
        #  - val: dict with name and value of implemented variable

        self.state_var_impl['n' + str(k)] = dict()
        # implementation of current state variable
        for name, val in self.state_var.items():
            var_impl = cs.SX.sym(name + '_' + str(k), val.shape[0])
            self.state_var_impl['n' + str(k)].update({name : var_impl})

        # implementation of past state variable, getting it from past nodes
        for name, val in self.state_var_prev.items():
            k_prev = int(name[name.index('-'):])
            if k >= -k_prev:
                var_name = name[:name.index('-')]
                self.state_var_impl['n' + str(k)].update({name: self.state_var_impl['n' + str(k + k_prev)][var_name]})

