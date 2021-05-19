import casadi as cs
from classes import function as fc
from classes import nodes as nd
from classes import state_variables as sv

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

    def createFunction(self, container, name, f, nodes=None):

        used_var = dict()
        if not nodes:
            nodes = [0, self.N]
        for name_var, value_var in self.state_var_container.getVarAbstrDict().items():
            if cs.depends_on(f, value_var):
                used_var[name_var] = value_var

        temp_fun = fc.Constraint(name, f, used_var, nodes)
        container.append(temp_fun)

        return temp_fun

    def createConstraint(self, name, g, nodes=None, bounds=None):

        container = self.cnstr_container
        fun = self.createFunction(container, name, g, nodes)

        return fun

    def createCostFunction(self, name, j, nodes=None):

        container = self.costfun_container
        fun = self.createFunction(container, name, j, nodes)

        return fun

    def _implementFunctions(self, container, node):
        f_impl = list()
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
        print('----------------------------------------------------')
        self.state_var_container.getVarImplList()

    # def setConstraint(self, cnstr):
    #     assert(isinstance(cnstr, fc.Constraint))
    #     self.cnstr_container.append(cnstr.getName())
