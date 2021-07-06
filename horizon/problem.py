import casadi as cs
from horizon import function as fc
from horizon import nodes as nd
from horizon import state_variables as sv
import numpy as np
import logging
import sys
import time

class Problem:

    def __init__(self, N, crash_if_suboptimal=False):

        self.logger = logging.getLogger('logger')
        # self.logger.setLevel(level=logging.DEBUG)
        self.debug_mode = self.logger.isEnabledFor(logging.DEBUG)
        stdout_handler = logging.StreamHandler(sys.stdout)
        self.logger.addHandler(stdout_handler)

        self.crash_if_suboptimal = crash_if_suboptimal

        self.nodes = N + 1
        # state variable to optimize
        self.state_var_container = sv.StateVariables(self.nodes)

        # just variables
        # todo one could set also non state variable right?
        self.var_container = list()

        # constraint variables
        self.cnstr_container = list()
        self.cnstr_impl = list()

        self.costfun_container = list()
        self.costfun_impl = list()

    def createStateVariable(self, name, dim, prev_nodes=None):
        var = self.state_var_container.setStateVar(name, dim, prev_nodes)
        return var

    def createInputVariable(self, name, dim, prev_nodes=None):
        var = self.state_var_container.setInputVar(name, dim, prev_nodes)
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
            nodes = [0, self.nodes]

        used_var = self._getUsedVar(g)
        if self.debug_mode:
            self.logger.debug('Creating function "{}": {} with abstract variables {}'.format(name, g, used_var))
        fun = fc.Constraint(name, g, used_var, nodes, bounds)
        container.append(fun)

        return fun

    def createCostFunction(self, name, j, nodes=None):

        container = self.costfun_container
        if not nodes:
            nodes = [0, self.nodes]

        used_var = self._getUsedVar(j)

        fun = fc.CostFunction(name, j, used_var, nodes)
        container.append(fun)

        return fun

    def removeCostFunction(self, name):

        if self.debug_mode:
            self.logger.debug('Functions before removal: {}'.format(self.costfun_container))
        for fun in self.costfun_container:
            if fun.getName() == name:
                self.costfun_container.remove(fun)
        if self.debug_mode:
            self.logger.debug('Function after removal: {}'.format(self.costfun_container))

    def removeConstraint(self, name):
        for fun in self.cnstr_container:
            if fun.getName() == name:
                self.cnstr_container.remove(fun)

    def setNNodes(self, n_nodes):
        self.nodes = n_nodes + 1
        self.state_var_container.setNNodes(self.nodes)

        for cnstr in self.cnstr_container:
            cnstr.setNodes([i for i in cnstr.getNodes() if i in self.nodes])

        for costfun in self.costfun_container:
            costfun.setNodes([i for i in costfun.getNodes() if i in self.nodes])

    def _implementFunctions(self, container, node):
        f_impl = list()

        # TODO be careful about ordering
        for fun in container:
            f = fun.getFunction()

            # implement constraint only if constraint is present in node k
            if node in fun.getNodes():
                used_vars = list()
                for name, val in fun.getVariables().items():
                    var = self.state_var_container.getVarImpl(name, node)
                    used_vars.append(var)

                f_impl.append(f(*used_vars))
                if self.debug_mode:
                    self.logger.debug('Implemented function "{}": {} with vars {}'.format(fun.getName(), f_impl, used_vars))
        return f_impl

    def _updateConstraints(self, node):
        temp_cnsrt_impl = self._implementFunctions(self.cnstr_container, node)
        if temp_cnsrt_impl:
            # add implemented constraints in list
            self.cnstr_impl += temp_cnsrt_impl


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

        for k in range(self.nodes):  # todo decide if N or N+1
            self.logger.debug('Node {}:'.format(k))
            # implement the abstract state variable with the current node
            self.state_var_container.update(k)
            # implement the constraint
            self._updateConstraints(k) #todo not sure but ok, maybe better a constraint class container that updates takin state_var_container?
            self._updateCostFunctions(k)
            self.logger.debug('===========================================')

            self.costfun_sum = cs.sum1(cs.vertcat(*self.costfun_impl))


        # self.logger.debug('state var unraveled:', self.state_var_container.getVarImplList())
        # self.logger.debug('constraints unraveled:', cs.vertcat(*self.cnstr_impl))
        # self.logger.debug('cost functions unraveled:', cs.vertcat(*self.costfun_impl))
        # self.logger.debug('cost function summed:', self.costfun_sum)
        # self.logger.debug('----------------------------------------------------')

        j = self.costfun_sum
        w = self.state_var_container.getVarImplList()
        g = cs.vertcat(*self.cnstr_impl)
        self.prob = {'f': j, 'x': w, 'g': g}

        self.solver = cs.nlpsol('solver', 'ipopt', self.prob,
                           {'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 3, 'sb': 'yes'},
                            'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3

    def solveProblem(self):

        # t_start = time.time()
        self.state_var_container.updateBounds()
        self.state_var_container.updateInitialGuess()

        w = self.state_var_container.getVarImplList()
        self.w0 = self.state_var_container.getInitialGuessList()

        if self.debug_mode:
            self.logger.debug('Initial guess vector for variables:'.format(self.state_var_container.getInitialGuessList()))

        g = cs.vertcat(*self.cnstr_impl)
        j = self.costfun_sum

        self.lbg = []
        self.ubg = []

        for node in range(self.nodes):
            for cnstr in self.cnstr_container:
                if node in cnstr.getNodes():
                    self.lbg += cnstr.getBoundsMin(node)
                    self.ubg += cnstr.getBoundsMax(node)

        self.lbw = self.state_var_container.getBoundsMinList()
        self.ubw = self.state_var_container.getBoundsMaxList()

        if self.debug_mode:
            self.logger.debug('================')
            self.logger.debug('len w: {}'.format(w.shape))
            self.logger.debug('len lbw: {}'.format(len(self.lbw)))
            self.logger.debug('len ubw: {}'.format(len(self.ubw)))
            self.logger.debug('len w0: {}'.format(len(self.w0)))
            self.logger.debug('len g: {}'.format(g.shape))
            self.logger.debug('len lbg: {}'.format(len(self.lbg)))
            self.logger.debug('len ubg: {}'.format(len(self.ubg)))


            self.logger.debug('================')
            self.logger.debug('w: {}'.format(w))
            self.logger.debug('lbw: {}'.format(self.lbw))
            self.logger.debug('ubw: {}'.format(self.ubw))
            self.logger.debug('g: {}'.format(g))
            self.logger.debug('lbg: {}'.format(self.lbg))
            self.logger.debug('ubg: {}'.format(self.ubg))
            self.logger.debug('j: {}'.format(j))

        # t_to_set_up = time.time() - t_start
        # print('T to set up:', t_to_set_up)
        # t_start = time.time()
        sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg)

        # t_to_solve = time.time() - t_start
        # print('T to solve:', t_to_solve)
        # t_start = time.time()

        if self.crash_if_suboptimal:
            if not self.solver.stats()['success']:
                raise Exception('Optimal solution NOT found.')

        self.w_opt = sol['x'].full().flatten()

        # split solution for each variable
        solution_dict = {name: np.zeros([var.shape[0], var.getNNodes()]) for name, var in self.state_var_container.getVarAbstrDict(past=False).items()}

        pos = 0

        for node, val in self.state_var_container.getVarImplDict().items():
            if self.debug_mode:
                self.logger.debug('Node: {}'.format(node))

            for name, var in val.items():
                dim = var['var'].shape[0]
                node_number = int(node[node.index('n') + 1:])
                solution_dict[name][:, node_number] = self.w_opt[pos:pos + dim]

                if self.debug_mode:
                    self.logger.debug('var {} of dim {}'.format(name, var['var'].shape[0]))
                    self.logger.debug('Previous state: {}'.format(solution_dict))
                    self.logger.debug('Var state: {}'.format(solution_dict[name]))
                    self.logger.debug('Appending to {} opt sol [{}-{}]: {}'.format(name, pos, pos + dim, sol['x']))
                    self.logger.debug('Current state: {}'.format(solution_dict))
                    # self.logger.debug('~~~~~~~~~~~~~')
                pos = pos + dim

        # t_to_finish = time.time() - t_start
        # print('T to finish:', t_to_finish)
        return solution_dict

    def serialize(self):

        self.state_var_container.serialize()

        self.cnstr_container[:] = [var.serialize() for var in self.cnstr_container]
        self.cnstr_impl[:] = [var.serialize() for var in self.cnstr_impl]

        self.costfun_container[:] = [var.serialize() for var in self.costfun_container]
        self.costfun_impl[:] = [var.serialize() for var in self.costfun_impl]

    def deserialize(self):

        self.state_var_container.deserialize()

        self.cnstr_container[:] = [var.deserialize() for var in self.cnstr_container]
        self.cnstr_impl[:] = [var.deserialize() for var in self.cnstr_impl]

        self.costfun_container[:] = [var.deserialize() for var in self.costfun_container]
        self.costfun_impl[:] = [var.deserialize() for var in self.costfun_impl]


if __name__ == '__main__':
    prb = Problem(10)
    x = prb.createStateVariable('x', 4)
    y = prb.createStateVariable('y', 4)

    danieli = prb.createConstraint('danieli', x+y)

    print(danieli.getDim())


