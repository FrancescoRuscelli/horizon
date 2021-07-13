import casadi as cs
from horizon import function as fc
from horizon import nodes as nd
from horizon import state_variables as sv
import numpy as np
import logging
import sys

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
        self.function_container = fc.FunctionsContainer(self.state_var_container, self.nodes, self.logger)

        # just variables
        # todo one could set also non state variable right?
        self.var_container = list()

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

        if not nodes:
            nodes = [0, self.nodes]

        used_var = self._getUsedVar(g)
        if self.debug_mode:
            self.logger.debug('Creating function "{}": {} with abstract variables {}'.format(name, g, used_var))
        fun = fc.Constraint(name, g, used_var, nodes, bounds)

        self.function_container.addFunction(fun)

        return fun

    def createCostFunction(self, name, j, nodes=None):

        if not nodes:
            nodes = [0, self.nodes]

        used_var = self._getUsedVar(j)

        fun = fc.CostFunction(name, j, used_var, nodes)

        self.function_container.addFunction(fun)

        return fun

    def removeCostFunction(self, name):

        # if self.debug_mode:
        #     self.logger.debug('Functions before removal: {}'.format(self.costfun_container))
        self.function_container.removeFunction(name)
        # if self.debug_mode:
        #     self.logger.debug('Function after removal: {}'.format(self.costfun_container))

    def removeConstraint(self, name):
        self.function_container.removeFunction(name)

    def setNNodes(self, n_nodes):
        self.nodes = n_nodes + 1
        self.state_var_container.setNNodes(self.nodes)
        self.function_container.setNNodes(self.nodes)

    def createProblem(self):
        # this is to reset both the constraints and the cost functions everytime I create a problem
        self.state_var_container.clear()
        self.function_container.clear()

        for k in range(self.nodes):  # todo decide if N or N+1
            self.logger.debug('Node {}:'.format(k))
            # implement the abstract state variable with the current node
            self.state_var_container.update(k)
            # implement the constraints and the cost functions with the current node
            self.function_container.update(k)

            self.logger.debug('===========================================')

        self.j = self.function_container.getCostFImplSum()
        self.w = self.state_var_container.getVarImplList()
        self.g = self.function_container.getCnstrFList()

        # self.logger.debug('state var unraveled:', self.state_var_container.getVarImplList())
        # self.logger.debug('constraints unraveled:', cs.vertcat(*self. ...))
        # self.logger.debug('cost functions unraveled:', cs.vertcat(*self. ...))
        # self.logger.debug('cost function summed:', self.j)
        # self.logger.debug('----------------------------------------------------')

        if self.debug_mode:
            self.logger.debug('cost fun: {}'.format(self.j))
            self.logger.debug('state variables: {}'.format(self.w))
            self.logger.debug('constraints: {}'.format(self.g))

        self.prob = {'f': self.j, 'x': self.w, 'g': self.g}

        self.solver = cs.nlpsol('solver', 'ipopt', self.prob,
                                {'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 3, 'sb': 'yes'},
                                 'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3

    def solveProblem(self):

        # t_start = time.time()
        self.state_var_container.updateBounds()
        self.state_var_container.updateInitialGuess()

        self.w0 = self.state_var_container.getInitialGuessList()

        if self.debug_mode:
            self.logger.debug('Initial guess vector for variables:'.format(self.state_var_container.getInitialGuessList()))

        self.lbw = self.state_var_container.getBoundsMinList()
        self.ubw = self.state_var_container.getBoundsMaxList()

        self.lbg = self.function_container.getLowerBoundsList()
        self.ubg = self.function_container.getUpperBoundsList()


        if self.debug_mode:
            self.logger.debug('================')
            self.logger.debug('len w: {}'.format(self.w.shape))
            self.logger.debug('len lbw: {}'.format(len(self.lbw)))
            self.logger.debug('len ubw: {}'.format(len(self.ubw)))
            self.logger.debug('len w0: {}'.format(len(self.w0)))
            self.logger.debug('len g: {}'.format(self.g.shape))
            self.logger.debug('len lbg: {}'.format(len(self.lbg)))
            self.logger.debug('len ubg: {}'.format(len(self.ubg)))


            self.logger.debug('================')
            self.logger.debug('w: {}'.format(self.w))
            self.logger.debug('lbw: {}'.format(self.lbw))
            self.logger.debug('ubw: {}'.format(self.ubw))
            self.logger.debug('g: {}'.format(self.g))
            self.logger.debug('lbg: {}'.format(self.lbg))
            self.logger.debug('ubg: {}'.format(self.ubg))
            self.logger.debug('j: {}'.format(self.j))

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

    def scopeNodeVars(self, node):

        return self.state_var_container.getVarImplAtNode(node)

    def scopeNodeConstraints(self, node):
        return self.function_container.getCnstrFImplAtNode(node)

    def scopeNodeCostFunctions(self, node):
        return self.function_container.getCostFImplAtNode(node)

    def serialize(self):

        self.state_var_container.serialize()
        self.function_container.serialize()

    def deserialize(self):

        self.state_var_container.deserialize()
        self.function_container.deserialize()


if __name__ == '__main__':
    prb = Problem(8)
    x = prb.createStateVariable('x', 2)
    y = prb.createStateVariable('y', 2)

    x.setBounds([-2, -2], [2, 2], [1, 5])

    # todo this is allright but I have to remember that if I update the nodes (from 3 to 6 for example) i'm not updating the constraint nodes
    # todo so if it was active on all the node before, then it will be active only on the node 1, 2, 3 (not on 4, 5, 6)


    scoping_node = 8
    # print('var at nodes {}  BEFORE creating the problem: '.format(scoping_node), prb.scopeNodeVars(scoping_node))
    # print('number of nodes of {}: {}'.format(x, x.getNNodes()))
    # print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))

    # print('getVarImplList way before:', prb.state_var_container.getVarImplList())
    danieli = prb.createConstraint('danieli', x+y)
    sucua = prb.createCostFunction('sucua', x*y)
    prb.createProblem()

    for i in range(8):
        print(x.getBounds(i))

    # print('var at nodes {}  AFTER creating the problem: '.format(scoping_node), prb.scopeNodeVars(scoping_node))
    # print('getVarImplList before:', prb.state_var_container.getVarImplList())
    new_n_nodes = 5
    # print('================== Changing n. of nodes to {} =================='.format(new_n_nodes))
    prb.setNNodes(new_n_nodes)
    scoping_node = 8
    # print('var at nodes {} AFTER changing the n. of nodes but BEFORE rebuilding: {}'.format(scoping_node, prb.scopeNodeVars(scoping_node)))
    # print('number of nodes of {}: {}'.format(x, x.getNNodes()))
    # print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))
    # print('getVarImplList after but before create:', prb.state_var_container.getVarImplList())
    prb.createProblem()

    # todo check why this is so
    # print('after:', prb.state_var_container.getVarImplList())

    scoping_node = 8
    # print('var at nodes {} AFTER changing the n. of nodes but AFTER rebuilding: {}'.format(scoping_node, prb.scopeNodeVars(scoping_node)))
    # print('number of nodes of {}: {}'.format(x, x.getNNodes()))
    # print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))
    # x.setBounds(10)
    # danieli.setNodes([1,6])
    prb.scopeNodeVars(2)


    x.setBounds([2,8], 4)

    # todo what do I do?
    # is it better to project the abstract variable as soon as it is created, to set the bounds and everything?
    # or is it better to wait for the buildProblem to generate the projection of the abstract value along the horizon line?





