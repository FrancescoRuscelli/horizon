import casadi as cs

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

    def getFunction(self):
        return self.fun

    def getNodes(self):
        return self.nodes

    def setNodes(self, nodes, erasing=False):

        # todo check this logic
        # check if  int, list or list of list
        if isinstance(nodes, int):
            unraveled_nodes = nodes
            pass
        elif any(isinstance(el, list) for el in nodes):
            unraveled_nodes = list()
            for el in nodes:
                temp = list(range(el[0], el[1]))
                for item in temp:
                    unraveled_nodes.append(item)
        elif isinstance(nodes, list):
            unraveled_nodes = list()
            temp = list(range(nodes[0], nodes[1]))
            for item in temp:
                unraveled_nodes.append(item)

        if erasing:
            self.nodes.clear()

        # adding to function nodes
        if isinstance(unraveled_nodes, int):
            if unraveled_nodes not in self.nodes:
                self.nodes.append(unraveled_nodes)
                self.nodes.sort()
        else:
            for i in unraveled_nodes:
                if i not in self.nodes:
                    self.nodes.append(i)
                    self.nodes.sort()

    def getVariables(self):
        return self.vars
        # return [var for name, var in self.var]

    def getType(self):
        return 'generic'

class Constraint(Function):
    def __init__(self, name, f, used_vars, nodes, bounds=None):
        super().__init__(name, f, used_vars, nodes)

        self.bounds = []
        # todo setBounds
        # self.setBounds(bounds)

    # todo transform string in typeFun "ConstraintType"
    def getType(self):
        return 'constraint'

    def setBounds(self):
        print('wip')

class CostFunction(Function):
    def __init__(self, name, f, used_vars, nodes):
        super().__init__(name, f, used_vars, nodes)

    def getType(self):
        return 'costfunction'




