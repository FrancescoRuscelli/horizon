import casadi as cs
import numpy as np
from horizon import misc_function as misc
from collections import OrderedDict
import pickle

class Function:
    def __init__(self, name, f, used_vars, nodes):

        self.f = f
        self.name = name
        self.nodes = []

        self.vars = used_vars #todo isn't there another way to get the variables from the function g?

        self.fun = cs.Function(name, list(self.vars.values()), [self.f])
        self.fun_impl = dict()

        self.setNodes(nodes)

    def getName(self):
        return self.name

    def getDim(self):
        return self.f.shape

    def getFunction(self):
        return self.fun

    def getFunctionImpl(self, node):
        if 'n' + str(node) in self.fun_impl:
            return self.fun_impl['n' + str(node)]
        else:
            return None

    def _project(self, node, used_vars):
        # print('projected fun "{}" with vars {}:'.format(self.fun, used_vars))
        self.fun_impl['n' + str(node)] = self.fun(*used_vars)
        return self.fun_impl['n' + str(node)]

    def getNodes(self):
        return self.nodes

    def setNodes(self, nodes, erasing=False):
        # unraveled_nodes = misc.unravelElements(nodes)

        if erasing:
            self.nodes.clear()

        # adding to function nodes
        for i in nodes: #unraveled_nodes:
            if i not in self.nodes:
                self.nodes.append(i)
                self.nodes.sort()

        print('nodes of function {} are: {}'.format(self.name, self.nodes))

    def getVariables(self):
        return self.vars
        # return [var for name, var in self.var]

    def getType(self):
        return 'generic'

    def serialize(self):

        self.f = self.f.serialize()

        for name, data in self.vars.items():
            self.vars[name] = data.serialize()

        for node, item in self.fun_impl.items():
            self.fun_impl[node] = item.serialize()

        self.fun = self.fun.serialize()

        return self

    def deserialize(self):

        self.f = cs.SX.deserialize(self.f)

        for name, data in self.vars.items():
            self.vars[name] = cs.SX.deserialize(data)

        for node, item in self.fun_impl.items():
            self.fun_impl[node] = cs.Function.deserialize(item)

        self.fun = cs.Function.deserialize(self.fun)

        return self


class Constraint(Function):
    def __init__(self, name, f, used_vars, nodes, bounds=None):

        self.bounds = dict()
        for node in nodes:
            self.bounds['n' + str(node)] = dict(lb=[-np.inf] * f.shape[0], ub=[np.inf] * f.shape[0])

        super().__init__(name, f, used_vars, nodes)

        # todo setBounds
        if bounds is not None:

            if 'nodes' not in bounds:
                bounds['nodes'] = None

            self.setBounds(lb=bounds['lb'], ub=bounds['ub'], nodes=bounds['nodes'])

    # todo transform string in typeFun "ConstraintType"
    def getType(self):
        return 'constraint'

    def setLowerBounds(self, bounds, nodes=None):

        print('sakamoto penis', self.nodes)
        if nodes is None:
            # unraveled_nodes
            nodes = self.nodes
        # else:
        #     unraveled_nodes = misc.unravelElements(nodes) # todo are we sure this is ok?

        for node in nodes: #unraveled_nodes:
            if node in self.nodes:
                self.bounds['n' + str(node)].update({'lb': bounds})

    def setUpperBounds(self, bounds, nodes=None):

        if nodes is None:
            # unraveled_nodes
            nodes = self.nodes
        # else:
        #     unraveled_nodes = misc.unravelElements(nodes)

        for node in nodes: #unraveled_nodes:
            if node in self.nodes:
                self.bounds['n' + str(node)].update({'ub': bounds})

    def setBounds(self, lb, ub, nodes=None):

        # todo wrong! check if lb and ub are the length of the variable!
        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def getLowerBounds(self, node):
        lb = self.bounds['n' + str(node)]['lb']
        return lb

    def getUpperBounds(self, node):
        ub = self.bounds['n' + str(node)]['ub']
        return ub

    def getBounds(self, nodes):
        return [self.getLowerBounds(nodes), self.getUpperBounds(nodes)]

    def setNodes(self, nodes, erasing=False):
        # unraveled_nodes = misc.unravelElements(nodes)

        if erasing:
            self.nodes.clear()

        # adding to function nodes
        for i in nodes: #unraveled_nodes:
            if i not in self.nodes:
                self.nodes.append(i)
                self.nodes.sort()
                if 'n' + str(i) not in self.bounds:
                    self.bounds['n' + str(i)] = dict(lb=[-np.inf] * self.f.shape[0], ub=[np.inf] * self.f.shape[0])

class CostFunction(Function):
    def __init__(self, name, f, used_vars, nodes):
        super().__init__(name, f, used_vars, nodes)

    def getType(self):
        return 'costfunction'

class FunctionsContainer:
    def __init__(self, state_vars, nodes, logger=None):

        self.logger = logger

        self.state_vars = state_vars
        self.nodes = nodes

        self.cnstr_container = OrderedDict()
        self.cnstr_impl = OrderedDict()

        self.costfun_container = OrderedDict()
        self.costfun_impl = OrderedDict()


    def addFunction(self, fun):
        if fun.getType() == 'constraint':
            self.cnstr_container[fun.getName()] = fun
        elif fun.getType() == 'costfunction':
            self.costfun_container[fun.getName()] = fun
        elif fun.getType() == 'generic':
            print('function.py: generic not implemented')

    def removeFunction(self, fun_name):
        if fun_name in self.cnstr_container:
            del self.cnstr_container[fun_name]
        elif fun_name in self.costfun_container:
            del self.costfun_container[fun_name]

    def getFunction(self, fun_name):
        if fun_name in self.cnstr_container:
            return self.cnstr_container[fun_name]
        elif fun_name in self.costfun_container:
            return self.costfun_container[fun_name]
        else:
            return None

    def getCnstrFDict(self):
        return self.cnstr_container

    def getCostFDict(self):
        return self.costfun_container

    def update(self, node):
        self.updateFun(node, self.cnstr_container, self.cnstr_impl)
        self.updateFun(node, self.costfun_container, self.costfun_impl)

    def updateFun(self, node, container, container_impl):

        container_impl['n' + str(node)] = dict()

        # TODO be careful about ordering
        for fun_name, fun in container.items():
            # print('implementing function "{}" at node {}.'.format(fun_name, node))
            # print('Function is active on nodes: {}'.format(fun.getNodes()))
            # print('Function depends on variables: {}'.format(fun.getVariables()))
            # implement constraint only if constraint is present in node k
            if node in fun.getNodes():
                used_vars = list()
                for name, val in fun.getVariables().items():
                    var = self.state_vars.getVarImpl(name, node)
                    # print('used abstract var "{}" implemented as {}'.format(name, var))
                    used_vars.append(var)


                f_impl = fun._project(node, used_vars)
                if fun.getType() == 'constraint':
                    lb = fun.getLowerBounds(node)
                    ub = fun.getUpperBounds(node)
                    fun_dict = dict(val=f_impl, lb=lb, ub=ub)
                else:
                    fun_dict = f_impl

                container_impl['n' + str(node)].update({fun_name: fun_dict})
                # print('==================================================')
                self.logger.debug('Implemented function "{}" of type {}: {} with vars {}'.format(fun_name, fun.getType(), f_impl, used_vars))

    def getCnstrFImpl(self, name, node):
        if 'n' + str(node) in self.cnstr_impl:
            if name in self.cnstr_impl['n' + str(node)]:
                return self.cnstr_impl['n' + str(node)][name]

    def getCostFImpl(self, name, node):
        if 'n' + str(node) in self.costfun_impl:
            if name in self.costfun_impl['n' + str(node)]:
                return self.costfun_impl['n' + str(node)][name]

    def getCnstrFImplAtNode(self, node):
        if 'n' + str(node) in self.cnstr_impl:
            return self.cnstr_impl['n' + str(node)]

    def getCostFImplAtNode(self, node):
        if 'n' + str(node) in self.costfun_impl:
            return self.costfun_impl['n' + str(node)]

    def getCostFImplDict(self):
        return self.costfun_impl

    def getCnstrFImplDict(self):
        return self.cnstr_impl

    def getCostFImplSum(self):
        costfun_impl = list()
        for node in self.costfun_impl.values():
            for elem in node.values():
                costfun_impl.append(elem)

        return cs.sum1(cs.vertcat(*costfun_impl))

    def getCnstrFList(self):
        cnstr_impl = list()
        for node in self.cnstr_impl.values():
            for elem in node.values():
                cnstr_impl.append(elem['val'])

        return cs.vertcat(*cnstr_impl)

    def getLowerBoundsList(self):
        lbg = list()
        for node in self.cnstr_impl.values():
            for elem in node.values():
                lbg += elem['lb']

        return lbg

    def getUpperBoundsList(self):
        ubg = list()
        for node in self.cnstr_impl.values():
            for elem in node.values():
                ubg += elem['ub']

        return ubg

    def setNNodes(self, n_nodes):
        # this is required to update the function_container EACH time a new number of node is set
        # todo check!
        self.nodes = n_nodes

        for cnstr in self.cnstr_container.values():
            cnstr.setNodes([i for i in cnstr.getNodes() if i in range(self.nodes)], erasing=True)

        for costfun in self.costfun_container.values():
            costfun.setNodes([i for i in costfun.getNodes() if i in range(self.nodes)], erasing=True)

    def getNCnstrFun(self):
        return len(self.cnstr_container)

    def getNCostFun(self):
        return len(self.costfun_container)

    def clear(self):
        self.cnstr_impl.clear()
        self.costfun_impl.clear()

    def serialize(self):

        for name, item in self.cnstr_container.items():
            self.cnstr_container[name] = item.serialize()

        for node, item in self.cnstr_impl.items():
            for name, elem in item.items():
                self.cnstr_impl[node][name]['val'] = elem['val'].serialize()

        for name, item in self.costfun_container.items():
            self.costfun_container[name] = item.serialize()

        for node, item in self.costfun_impl.items():
            for name, elem in item.items():
                self.costfun_impl[node][name] = elem.serialize()


    def deserialize(self):

        for name, item in self.cnstr_container.items():
            self.cnstr_container[name] = item.deserialize()

        for node in self.cnstr_impl.keys():
            for name, item in self.cnstr_impl[node].items():
                self.cnstr_impl[node][name]['val'] = cs.SX.deserialize(item['val'])

        # these are CASADI functions
        for name, item in self.costfun_container.items():
            self.costfun_container[name] = item.deserialize()

        # these are SX variables
        for node, item in self.costfun_impl.items():
            for name, elem in item.items():
                self.costfun_impl[node][name] = cs.SX.deserialize(elem)





if __name__ == '__main__':

    x = cs.SX.sym('x', 2)
    y = cs.SX.sym('y', 2)
    fun = x+y
    used_var = dict(x=x, y=y)
    funimpl = Function('dan', fun, used_var, 1)

    funimpl = funimpl.serialize()
    print('===PICKLING===')
    funimpl_serialized = pickle.dumps(funimpl)
    print(funimpl_serialized)
    print('===DEPICKLING===')
    funimpl_new = pickle.loads(funimpl_serialized)


