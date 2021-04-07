import casadi as cs
import pprint
from collections import OrderedDict

def flatten(l):
    if isinstance(l, list):
        for e in l:
            yield from flatten(e)
    else:
        yield l

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
            return  var
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
            if k>0:
                k_prev = int(name[name.index('-'):])
                var_name = name[:name.index('-')]
                self.state_var_impl['n' + str(k)].update({name: self.state_var_impl['n' + str(k + k_prev)][var_name]})

class Node:

    def __init__(self, value, lb=None, ub=None):

        # todo check if are good numbers

        self.value = value

        if lb is None:
            lb = -cs.inf

        if ub is None:
            ub = cs.inf

        self.lb = lb
        self.ub = ub

class Constraint:

    def __init__(self, name, g, used_vars, nodes):

        self.g = g
        self.name = name
        self.nodes = []
        self.vars = used_vars #todo isn't there another way to get the variable from the function g

        self.fun = cs.Function(name, list(self.vars.values()), [self.g])

        for i in flatten(nodes):
            self.nodes.append(Node(i))

    def getName(self):
        return self.name

    def getFunction(self):
        return self.fun

    def getNodes(self):
        return self.nodes

    # def setNodes(self):
        # todo

    def getVariables(self):
        return self.vars
        # return [var for name, var in self.var]

class Problem:

    def __init__(self, N, crash_if_suboptimal=False):

        self.crash_if_suboptimal = crash_if_suboptimal

        self.N = N
        # state variable to optimize
        self.state_var_container = StateVariables()

        # just variables
        self.var_container = list()
        # constraint variables
        self.cnstr_container = list()
        self.cnstr_impl = list()

    def createStateVariable(self, name, dim, prev_nodes=None):
        var = self.state_var_container.setVar(name, dim, prev_nodes)
        return var

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

    def createConstraint(self, name, g, nodes=None):

        used_var = dict()
        if not nodes:
            nodes = self.N
        for name_var, value_var in self.state_var_container.getVarAbstrDict().items():
            if cs.depends_on(g, value_var):
                used_var[name_var] = value_var

        temp_cnstr = Constraint(name, g, used_var, nodes)
        self.cnstr_container.append(temp_cnstr)

        return temp_cnstr

    def _updateConstraints(self, k):

        # print('name:', name)
        # print('f:', self.g_dict[name]['constraint'])
        # print('var_opt:', self.var_opt)
        # print('vars:', [self.var_opt[x] for x in self.g_dict[name]['var']])
        # print('g_dict:', self.g_dict[name])
        for cnsrt in self.cnstr_container:
            f = cnsrt.getFunction()
            g = f(*[self.state_var_container.getVarImpl(name, k) for name, val in cnsrt.getVariables().items()])
            self.cnstr_impl.append(g)
        # print('g: {} {}'.format(name, g.shape))
        # print('value:', g)
        # print('bounds: {}'.format(self.g_dict[name]['bounds']))
    # def getVariablesName(self):
    #     return [name for name, var in self.var]
    def createProblem(self):

        for k in range(self.N):  # todo decide if N or N+1
            # implement the abstract state variable with the current node
            self.state_var_container.update(k)
            # implement the constraint
            self._updateConstraints(k) #todo not sure but ok, maybe better a constraint class container that updates takin state_var_container?

        print(cs.vertcat(*self.cnstr_impl))
        self.state_var_container.getVarImplList()


    def setConstraint(self, cnstr):
        assert(isinstance(cnstr, Constraint))
        self.cnstr_container.append(cnstr.getName())

if __name__ == '__main__':

    prb = Problem(4)

    x = prb.createStateVariable('x', 6)
    x_prev = prb.createStateVariable('x', 6, prev_nodes=-1) # how to do for previous nodes?
    u = prb.createStateVariable('u', 4)
    z = prb.createStateVariable('z', 2)

    print(x)
    print(x_prev)
    print(u)
    print(z)

    # todo return a variable so that it can be used instead of class attributes
    # todo saving a variable (or a function) inside the problem is probably completely useless if it's not a STATE variable

    # todo how to check if a function has variable that are not defined in the problem? (they are not in state_var)

    state_fun = x_prev[2:6] + u
    fun = u[2:4] +  z **2

    # prb.setVariable('state_fun', state_fun) # is it ok?
    # prb.setVariable('fun', fun)

    # cnsrt = prb.createConstraint('generic_constraint', x[0:2] - x[4:6], nodes=[[0,2], [3,4]])
    cnsrt = prb.createConstraint('another_constraint', u[0:2] - x[4:6], nodes=2)

    prb.createProblem()
    print('constraint nodes:', cnsrt.getNodes())
    print('constraint name:', cnsrt.getName())
    print('constraint function:', cnsrt.getFunction())
    print('constraint variables:', cnsrt.getVariables())

    exit()
    print(cnsrt.getNodes()[2].lb)
    print(cnsrt.getNodes()[2].value)

    # sz_iw

    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]], bounds=(dict(nodes=[0,2]))) # should be wrong
    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]], bounds=(dict(ubg=[1, 1])))