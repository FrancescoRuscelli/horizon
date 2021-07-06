import casadi as cs
import numpy as np
from horizon import misc_function as misc

class Function:
    def __init__(self, name, f, used_vars, nodes):

        self.f = f
        self.name = name
        self.nodes = []

        self.vars = used_vars #todo isn't there another way to get the variables from the function g?

        self.fun = cs.Function(name, list(self.vars.values()), [self.f])

        self.setNodes(nodes)

    def getName(self):
        return self.name

    def getDim(self):
        return self.f.shape

    def getFunction(self):
        return self.fun

    def getNodes(self):
        return self.nodes

    def setNodes(self, nodes, erasing=False):
        unraveled_nodes = misc.unravelElements(nodes)

        if erasing:
            self.nodes.clear()

        # adding to function nodes
        for i in unraveled_nodes:
            if i not in self.nodes:
                self.nodes.append(i)
                self.nodes.sort()

    def getVariables(self):
        return self.vars
        # return [var for name, var in self.var]

    def getType(self):
        return 'generic'

    def serialize(self):

        self.f = self.f.serialize()

        for name, data in self.vars.items():
            self.vars[name] = data.serialize()

        self.fun = self.fun.serialize()

        return self

    def deserialize(self):

        self.f = cs.SX.deserialize(self.f)

        for name, data in self.vars.items():
            self.vars[name] = cs.SX.deserialize(data)

        self.fun = cs.Function.deserialize(self.fun)

        return self

class Constraint(Function):
    def __init__(self, name, f, used_vars, nodes, bounds=None):
        super().__init__(name, f, used_vars, nodes)

        self.bounds = dict()
        for node in self.nodes:
            self.bounds['n' + str(node)] = dict(lb=[-np.inf] * f.shape[0], ub=[np.inf] * f.shape[0])

        # todo setBounds
        if bounds is not None:

            if 'nodes' not in bounds:
                bounds['nodes'] = None

            self.setBounds(lb=bounds['lb'], ub=bounds['ub'], nodes=bounds['nodes'])


    # todo transform string in typeFun "ConstraintType"
    def getType(self):
        return 'constraint'

    def setLowerBounds(self, bounds, nodes=None):

        if nodes is None:
            unraveled_nodes = self.nodes
        else:
            unraveled_nodes = misc.unravelElements(nodes)

        for node in unraveled_nodes:
            if node in self.nodes:
                self.bounds['n' + str(node)].update({'lb': bounds})

    def setUpperBounds(self, bounds, nodes=None):

        if nodes is None:
            unraveled_nodes = self.nodes
        else:
            unraveled_nodes = misc.unravelElements(nodes)

        for node in unraveled_nodes:
            if node in self.nodes:
                self.bounds['n' + str(node)].update({'ub': bounds})

    def setBounds(self, lb, ub, nodes=None):

        self.setLowerBounds(lb, nodes)
        self.setUpperBounds(ub, nodes)

    def getLowerBounds(self, node):
        print(self.bounds)
        lb = self.bounds['n' + str(node)]['lb']
        return lb

    def getUpperBounds(self, node):
        ub = self.bounds['n' + str(node)]['ub']
        return ub

    def getBounds(self, nodes):
        return [self.getLowerBounds(nodes), self.getUpperBounds(nodes)]

class CostFunction(Function):
    def __init__(self, name, f, used_vars, nodes):
        super().__init__(name, f, used_vars, nodes)

    def getType(self):
        return 'costfunction'




