import casadi as cs
from horizon import function as fc
from horizon import nodes as nd
from horizon import state_variables as sv
import numpy as np
import logging
import sys
import pickle

class Problem:

    def __init__(self, N, crash_if_suboptimal=False, logging_level=logging.INFO):

        self.opts = None
        self.solver = None
        self.logger = logging.getLogger('logger')
        self.logger.setLevel(level=logging_level)
        self.debug_mode = self.logger.isEnabledFor(logging.DEBUG)
        stdout_handler = logging.StreamHandler(sys.stdout)
        self.logger.addHandler(stdout_handler)

        self.crash_if_suboptimal = crash_if_suboptimal

        self.nodes = N + 1
        # state variable to optimize
        self.state_var_container = sv.StateVariables(self.nodes)
        self.function_container = fc.FunctionsContainer(self.state_var_container, self.nodes, self.logger)
        self.prob = None
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
        if nodes is None:
            nodes = list(range(0, self.nodes))
        else:
            if isinstance(nodes, list):
                nodes = [node for node in nodes if node in range(self.nodes)]
            else:
                nodes = [nodes] if nodes in range(self.nodes) else []


        used_var = self._getUsedVar(g)

        if self.debug_mode:
            self.logger.debug('Creating Constraint Function "{}": {} with abstract variables {}, active in nodes: {}'.format(name, g, used_var, nodes))

        fun = fc.Constraint(name, g, used_var, nodes, bounds)

        self.function_container.addFunction(fun)

        return fun

    def createCostFunction(self, name, j, nodes=None):

        if not nodes:
            nodes = list(range(self.nodes))
        else:
            if isinstance(nodes, list):
                nodes = [node for node in nodes if node in range(self.nodes)]
            else:
                nodes = [nodes] if nodes in range(self.nodes) else []

        used_var = self._getUsedVar(j)

        if self.debug_mode:
            self.logger.debug('Creating Cost Function "{}": {} with abstract variables {},  active in nodes: {}'.format(name, j, used_var, nodes))

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
        self.nodes = n_nodes + 1 # todo because I decided so
        self.state_var_container.setNNodes(self.nodes)
        self.function_container.setNNodes(self.nodes)

    def createProblem(self, opts=None):

        if opts is not None:
            self.opts = opts

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

        j = self.function_container.getCostFImplSum()
        w = self.state_var_container.getVarImplList()
        g = self.function_container.getCnstrFList()

        # self.logger.debug('state var unraveled:', self.state_var_container.getVarImplList())
        # self.logger.debug('constraints unraveled:', cs.vertcat(*self. ...))
        # self.logger.debug('cost functions unraveled:', cs.vertcat(*self. ...))
        # self.logger.debug('cost function summed:', self.j)
        # self.logger.debug('----------------------------------------------------')

        if self.debug_mode:
            self.logger.debug('cost fun: {}'.format(j))
            self.logger.debug('state variables: {}'.format(w))
            self.logger.debug('constraints: {}'.format(g))

        self.prob = {'f': j, 'x': w, 'g': g}

        if self.opts is not None:
            if "nlpsol.ipopt" in self.opts:
                self.solver = cs.nlpsol('solver', 'ipopt', self.prob)#,
                                    #{'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 3, 'sb': 'yes'},
                                    # 'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3

    def getProblem(self):
        return self.prob

    def setSolver(self, solver):
        self.solver = solver

    def getSolver(self):
        return self.solver

    def solveProblem(self):
        # t_start = time.time()
        if self.solver is None:
            self.logger.warning('Problem is not created. Nothing to solve!')
            return 0

        self.state_var_container.updateBounds()
        self.state_var_container.updateInitialGuess()

        w0 = self.state_var_container.getInitialGuessList()

        if self.debug_mode:
            self.logger.debug('Initial guess vector for variables: {}'.format(self.state_var_container.getInitialGuessList()))


        lbw = self.state_var_container.getBoundsMinList()
        ubw = self.state_var_container.getBoundsMaxList()

        lbg = self.function_container.getLowerBoundsList()
        ubg = self.function_container.getUpperBoundsList()


        if self.debug_mode:

            j = self.function_container.getCostFImplSum()
            w = self.state_var_container.getVarImplList()
            g = self.function_container.getCnstrFList()

            self.logger.debug('================')
            self.logger.debug('len w: {}'.format(w.shape))
            self.logger.debug('len lbw: {}'.format(len(lbw)))
            self.logger.debug('len ubw: {}'.format(len(ubw)))
            self.logger.debug('len w0: {}'.format(len(w0)))
            self.logger.debug('len g: {}'.format(g.shape))
            self.logger.debug('len lbg: {}'.format(len(lbg)))
            self.logger.debug('len ubg: {}'.format(len(ubg)))


            self.logger.debug('================')
            self.logger.debug('w: {}'.format(w))
            self.logger.debug('lbw: {}'.format(lbw))
            self.logger.debug('ubw: {}'.format(ubw))
            self.logger.debug('g: {}'.format(g))
            self.logger.debug('lbg: {}'.format(lbg))
            self.logger.debug('ubg: {}'.format(ubg))
            self.logger.debug('j: {}'.format(j))

        # t_to_set_up = time.time() - t_start
        # print('T to set up:', t_to_set_up)
        # t_start = time.time()

        sol = self.solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

        # t_to_solve = time.time() - t_start
        # print('T to solve:', t_to_solve)
        # t_start = time.time()

        if self.crash_if_suboptimal:
            if not self.solver.stats()['success']:
                raise Exception('Optimal solution NOT found.')

        w_opt = sol['x'].full().flatten()

        # split solution for each variable
        solution_dict = {name: np.zeros([var.shape[0], var.getNNodes()]) for name, var in self.state_var_container.getVarAbstrDict(past=False).items()}

        pos = 0
        self.logger.debug("SOLVER SOLUTION:")
        for node, val in self.state_var_container.getVarImplDict().items():
            if self.debug_mode:
                self.logger.debug('Node: {}'.format(node))

            for name, var in val.items():
                dim = var['var'].shape[0]
                node_number = int(node[node.index('n') + 1:])
                solution_dict[name][:, node_number] = w_opt[pos:pos + dim]
                pos = pos + dim

        if self.debug_mode:
            self.logger.debug('Current state: {}'.format(solution_dict))

        # t_to_finish = time.time() - t_start
        # print('T to finish:', t_to_finish)
        return solution_dict

    def getStateVariables(self):
        return self.state_var_container.getVarAbstrDict()

    def getConstraints(self):
        return self.function_container.getCnstrFDict()

    def scopeNodeVars(self, node):

        return self.state_var_container.getVarImplAtNode(node)

    def scopeNodeConstraints(self, node):
        return self.function_container.getCnstrFImplAtNode(node)

    def scopeNodeCostFunctions(self, node):
        return self.function_container.getCostFImplAtNode(node)

    def serialize(self):

        self.state_var_container.serialize()
        self.function_container.serialize()


        if self.prob:

            # self.prob.clear()
            # print('serializing f (type: {}): {}'.format(type(self.prob['f']), self.prob['f']))
            # print('serializing x (type: {}): {}'.format(type(self.prob['x']), self.prob['x']))
            # print('serializing g (type: {}): {}'.format(type(self.prob['g']), self.prob['g']))

            self.prob['f'] = self.prob['f'].serialize()
            self.prob['x'] = self.prob['x'].serialize()
            self.prob['g'] = self.prob['g'].serialize()

            # print('serialized f (type: {}): {}'.format(type(self.prob['f']), self.prob['f']))
            # print('serialized x (type: {}): {}'.format(type(self.prob['x']), self.prob['x']))
            # print('serialized g (type: {}): {}'.format(type(self.prob['g']), self.prob['g']))


        return self

    def deserialize(self):

        self.state_var_container.deserialize()
        self.function_container.deserialize()

        if self.prob:
            self.prob['f'] = cs.Sparsity.deserialize(self.prob['f']) if self.function_container.getNCostFun() == 0 else cs.SX.deserialize(self.prob['f'])
            self.prob['x'] = cs.SX.deserialize(self.prob['x'])
            self.prob['g'] = cs.Sparsity.deserialize(self.prob['g']) if self.function_container.getNCnstrFun() == 0 else cs.SX.deserialize(self.prob['g'])

            # print('deserializing f', self.prob['f'])
            # print('deserializing x', self.prob['x'])
            # print('deserializing g', self.prob['g'])

        return self


if __name__ == '__main__':

    # nodes = 8
    # prb = Problem(nodes)
    # x = prb.createStateVariable('x', 2)
    # y = prb.createStateVariable('y', 2)
    # danieli = prb.createConstraint('danieli', x+y)
    #
    # danieli.setBounds([12, 12],[12, 12], 4)
    # prb.createProblem()
    # sol = prb.solveProblem()
    #
    # print(sol)
    # exit()
    #
    #
    # print('===PICKLING===')
    # prb = prb.serialize()
    # prb_serialized = pickle.dumps(prb)
    #
    # print('===DEPICKLING===')
    # prb_new = pickle.loads(prb_serialized)
    # prb_new.deserialize()
    #
    # sv_new = prb_new.getStateVariables()
    # cnstr_new = prb_new.getConstraints()
    #
    # # these two are different
    # print('x', x)
    # print('new x', sv_new['x'])
    #
    # # todo how to check if the new state variable x is used by all the constraints?
    #
    # # oebus = prb_new.createConstraint('oebus', x)  # should not work
    # oebus = prb_new.createConstraint('oebus', sv_new['x'])  # should work
    # prb_new.createProblem()
    #
    # exit()
    # ==================================================================================================================
    # ==================================================================================================================
    # ==================================================================================================================
    # nodes = 8
    # prb = Problem(nodes)
    # x = prb.createStateVariable('x', 2)
    # y = prb.createStateVariable('y', 2)
    # # todo something wrong here
    # danieli = prb.createConstraint('danieli', x+y)
    # sucua = prb.createCostFunction('sucua', x*y)
    #
    #
    # prb.createProblem()
    #
    # print('===PICKLING===')
    #
    # prb = prb.serialize()
    # prb_serialized = pickle.dumps(prb)
    #
    #
    # print('===DEPICKLING===')
    # prb_new = pickle.loads(prb_serialized)
    # prb_new.deserialize()
    #
    # prb_new.createProblem()
    # print(prb_new.prob)
    #
    # exit()
    # ==================================================================================================================
    # ==================================================================================================================
    # ==================================================================================================================

    nodes = 8
    prb = Problem(nodes, logging_level=logging.INFO)
    x = prb.createStateVariable('x', 2)
    y = prb.createStateVariable('y', 2)

    x.setBounds([-2, -2], [2, 2])

    # todo this is allright but I have to remember that if I update the nodes (from 3 to 6 for example) i'm not updating the constraint nodes
    # todo so if it was active on all the node before, then it will be active only on the node 1, 2, 3 (not on 4, 5, 6)


    scoping_node = nodes
    # print('var at nodes {}  BEFORE creating the problem: '.format(scoping_node), prb.scopeNodeVars(scoping_node))
    # print('number of nodes of {}: {}'.format(x, x.getNNodes()))
    # print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))

    # print('getVarImplList way before:', prb.state_var_container.getVarImplList())
    danieli = prb.createConstraint('danieli', x+y)
    sucua = prb.createCostFunction('sucua', x*y, nodes=list(range(3, 15)))
    pellico = prb.createCostFunction('pellico', x-y, nodes=[0, 4, 6])

    danieli.setBounds(lb=[-1, -1], ub=[1,1], nodes=[1, 3])

    prb.createProblem()

    for i in range(nodes+1):
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
    # print('var at nodes {} AFTER changing the n. of nodes but AFTER rebuilding: {}'.format(scoping_node, prb.scopeNodeVars(scoping_node))) # should not work
    # print('number of nodes of {}: {}'.format(x, x.getNNodes()))
    # print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))
    # x.setBounds(10)
    # danieli.setNodes([1,6])
    prb.scopeNodeVars(2)


    x.setBounds([2, 8], [2, 8], 5)

    for i in range(new_n_nodes+1):
        print(x.getBounds(i))

    # todo what do I do?
    # is it better to project the abstract variable as soon as it is created, to set the bounds and everything?
    # or is it better to wait for the buildProblem to generate the projection of the abstract value along the horizon line?






