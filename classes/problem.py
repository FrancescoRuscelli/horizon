import casadi as cs
from classes import function as fc
from classes import nodes as nd
from classes import state_variables as sv
import numpy as np

class Problem:

    def __init__(self, N, crash_if_suboptimal=False):

        self.crash_if_suboptimal = crash_if_suboptimal

        self.N = N
        # state variable to optimize
        self.state_var_container = sv.StateVariables()

        # just variables
        self.var_container = list()
        # constraint variables
        self.cnstr_container = list()
        self.cnstr_impl = list()

        self.costfun_container = list()
        self.costfun_impl = list()

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

    def _getUsedVar(self, f):
        used_var = dict()
        for name_var, value_var in self.state_var_container.getVarAbstrDict().items():
            if cs.depends_on(f, value_var):
                used_var[name_var] = value_var

        return used_var

    # @classmethod
    # def createFunction(self, fun_type, **kwargs):
    #         return self.function[fun_type](**kwargs)

    def createConstraint(self, name, g, nodes=None, bounds=None):

        container = self.cnstr_container

        if not nodes:
            nodes = [0, self.N]

        used_var = self._getUsedVar(g)

        fun = fc.Constraint(name, g, used_var, nodes)
        container.append(fun)

        return fun

    def createCostFunction(self, name, j, nodes=None):

        container = self.costfun_container
        if not nodes:
            nodes = [0, self.N]

        used_var = self._getUsedVar(j)

        fun = fc.CostFunction(name, j, used_var, nodes)
        container.append(fun)

        return fun

    def removeCostFunction(self, name):

        print(self.costfun_container)
        for fun in self.costfun_container:
            if fun.getName() == name:
                self.costfun_container.remove(fun)

        print(self.costfun_container)

    def removeConstraint(self, name):
        for fun in self.cnstr_container:
            if fun.getName() == name:
                self.cnstr_container.remove(fun)

    def _implementFunctions(self, container, node):
        f_impl = list()

        # TODO be careful about ordering
        for fun in container:
            f = fun.getFunction()
            # implement constraint only if constraint is present in node k
            if node in fun.getNodes():
                f_impl.append(f(*[self.state_var_container.getVarImpl(name, node) for name, val in fun.getVariables().items()]))

        return f_impl

    def _updateConstraints(self, node):

        # print('name:', name)
        # print('f:', self.g_dict[name]['constraint'])
        # print('var_opt:', self.var_opt)
        # print('vars:', [self.var_opt[x] for x in self.g_dict[name]['var']])
        # print('g_dict:', self.g_dict[name])
        temp_cnsrt_impl = self._implementFunctions(self.cnstr_container, node)
        if temp_cnsrt_impl:
            # add implemented constraints in list
            self.cnstr_impl += temp_cnsrt_impl

        # print('g: {} {}'.format(name, g.shape))
        # print('value:', g)
        # print('bounds: {}'.format(self.g_dict[name]['bounds']))

    # def getVariablesName(self):
    #     return [name for name, var in self.var]

    def _updateCostFunctions(self, node):

        temp_costfun_impl = self._implementFunctions(self.costfun_container, node)
        if temp_costfun_impl:
            # add implemented cost function in list
            self.costfun_impl += temp_costfun_impl

    # todo add setStateBoundsFromName
    # def setStateBoundsFromName(self, name, ubw, lbw, nodes=None):

    def createProblem(self):

        for k in range(self.N):  # todo decide if N or N+1
            # implement the abstract state variable with the current node
            self.state_var_container.update(k)
            # implement the constraint
            self._updateConstraints(k) #todo not sure but ok, maybe better a constraint class container that updates takin state_var_container?
            self._updateCostFunctions(k)


            self.costfun_sum = cs.sum1(cs.vertcat(*self.costfun_impl))

        # print('state var unraveled:', self.state_var_container.getVarImplList())
        # print('constraints unraveled:', cs.vertcat(*self.cnstr_impl))
        # print('cost functions unraveled:', cs.vertcat(*self.costfun_impl))
        # print('cost function summed:', self.costfun_sum)
        # print('----------------------------------------------------')
        self.state_var_container.getVarImplList()

    def solveProblem(self):
        w = self.state_var_container.getVarImplList()
        w0 = np.zeros((w.shape[0]))
        g = cs.vertcat(*self.cnstr_impl)
        j = self.costfun_sum
        print('================')
        print('len w:', w.shape)
        # print('len lbw:', len(self.lbw))
        # print('len ubw:', len(self.ubw))
        print('len w0:', len(w0))
        print('len g:', g.shape)
        # print('len lbg:', len(self.ct.lbg))
        # print('len ubg:', len(self.ct.ubg))

        print('================')
        print('w:', w)
        # print('lbw:', self.lbw)
        # print('ubw:', self.ubw)
        print('g:', g)
        # print('lbg:', self.ct.lbg)
        # print('ubg:', self.ct.ubg)
        print('j:', j)


        # print(self.)
        # sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw, lbg=self.ct.lbg, ubg=self.ct.ubg)
        #
        # if self.crash_if_suboptimal:
        #     if not self.solver.stats()['success']:
        #         raise Exception('Optimal solution NOT found.')
        #
        # w_opt = sol['x'].full().flatten()
        #
        # return w_opt



# Problem.function = {
#     'constraint': Problem.createConstraint,
#     'cost_function': Problem.createCostFunction,
# }
    # def setConstraint(self, cnstr):
    #     assert(isinstance(cnstr, fc.Constraint))
    #     self.cnstr_container.append(cnstr.getName())
