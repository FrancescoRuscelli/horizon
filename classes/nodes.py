import casadi as cs

class Nodes:

    def __init__(self, value, lb=None, ub=None):

        # todo check if are good numbers

        self.value = value

        if lb is None:
            lb = -cs.inf

        if ub is None:
            ub = cs.inf

        self.lb = lb
        self.ub = ub

    def getValue(self):
        return self.value