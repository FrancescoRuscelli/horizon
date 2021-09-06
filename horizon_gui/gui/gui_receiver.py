from horizon import problem as horizon
import parser
import re
import casadi as cs
import math
import pickle
from logging import INFO, DEBUG
from horizon.utils import plotter as plt

class horizonImpl():
    def __init__(self, nodes, logger=None):

        # #todo logger! use it everywhere!
        self.logger = logger

        self.nodes = nodes
        self.casadi_prb = horizon.Problem(self.nodes, logging_level=DEBUG)

        self.sv_dict = dict()  # state variables
        self.fun_dict = dict() # functions

        self.plt = plt.PlotterHorizon(self.casadi_prb)
        # self.active_fun_list = list()

    def _setVarGenerator(self, var_type):
        if var_type == 'state_var':
            return self.casadi_prb.createStateVariable
        if var_type == 'input_var':
            return self.casadi_prb.createInputVariable
        if var_type == 'single_var':
            return self.casadi_prb.createSingleVariable
        if var_type == 'custom_var':
            return self.casadi_prb.createVariable

    def createVariable(self, var_type, name, dim, offset, nodes=None):

        flag, signal = self.checkVariable(name)

        if flag:

            if offset == 0:
                if var_type == 'state_var':
                    var = self.casadi_prb.createStateVariable(name, dim)
                if var_type == 'input_var':
                    var = self.casadi_prb.createInputVariable(name, dim)
                if var_type == 'single_var':
                    var = self.casadi_prb.createSingleVariable(name, dim)
                if var_type == 'custom_var':
                    var = self.casadi_prb.createVariable(name, dim, nodes)
            else:
                raise Exception('TBD prev/next state variables')

            self.sv_dict[name] = dict(var=var, dim=dim)

            return True, signal + f'. Type: {type(var)}'
        else:
            return False, signal

    def addFunction(self, data):

        # get Function from Text
        name = data['name']
        str_fun = data['str']

        flag, signal = self.checkFunction(name, str_fun)
        if flag:
            flag_syntax = self._createAndAppendFun(name, str_fun)
            if flag_syntax:
                return True, signal
            else:
                return False, "Syntax is wrong."
        else:
            return False, signal

    def activateFunction(self, name, fun_type):

        flag, signal = self.checkActiveFunction(name)

        active_nodes = None
        for var in self.fun_dict[name]['used_vars']:
            print(var.getNodes())


        if flag:
            if fun_type == 'constraint':
                try:
                    # self.logger.info('Adding function: {}'.format(self.fun_dict[name]))
                    active_fun = self.casadi_prb.createConstraint(name, self.fun_dict[name]['fun'])
                    # self.active_fun_list.append(active_fun)
                    self.fun_dict[name].update({'active': active_fun})
                except Exception as e:
                    return False, e

            elif fun_type == 'costfunction':
                try:
                    active_fun = self.casadi_prb.createCostFunction(name, self.fun_dict[name]['fun'])
                    self.fun_dict[name].update({'active': active_fun})
                except Exception as e:
                    return False, e

            return True, signal + 'Function "{}" activated as "{}".'.format(name, active_fun.getType())
        else:
            return False, signal

    def removeActiveFunction(self, name):

        active_fun_type = self.fun_dict[name]['active'].getType()
        self.fun_dict[name]['active'] = None

        if active_fun_type == 'constraint':
            self.casadi_prb.removeConstraint(name)
        elif active_fun_type == 'costfunction':
            self.casadi_prb.removeCostFunction(name)
        else:
            return False, 'Function type "{}" not recognized'.format(active_fun_type)

        return True, 'Function "{}" successfully removed.'.format(name)

    def removeStateVariable(self, data):
        print('"removeStateVariable" yet to implement. Data: {}'.format(data))

    def checkActiveFunction(self, name):

        if self.fun_dict[name]['active'] is not None:
            signal = "active function already inserted"
            return False, signal

        return True, 'Function "{}" can be activated. Adding.'.format(name)

    def checkFunction(self, name, fun): # fun, name

        if name in self.fun_dict.keys():
            signal = "function already Inserted"
            return False, signal

        elif name == "":
            signal = "Empty Name of Function Not Allowed"
            return False, signal

        if fun == "":
            signal = "Empty Function Not Allowed"
            return False, signal

        return True, 'Function "{}" is acceptable. Adding..'.format(name)

    def checkVariable(self, name):

        if name == "":
            signal = "Variable: Empty Value Not Allowed"
            return False, signal
        elif name in self.sv_dict.keys():
            signal = "Variable: Already Inserted"
            return False, signal
        elif name.isnumeric():
            signal = "Variable: Invalid Name"
            return False, signal

        return True, "Variable: generated '{}'".format(name)


    def fromTxtToFun(self, str_fun):

        fun = None
        # todo CHECK IF STR_FUN IS CORRECT?

        # todo better approach? the problem is that i should have here a set of variables
        # i don't want to write on the GUI self.x or worse self.horizon_receiver.sv_dict[]... better ideas?
        # is it possible for some of the variables not to be substituted?

        # get from text all variables and substitute them with self.horizon_receiver.sv_dict[''] ..

        # todo add also generic functions
        dict_vars = dict()
        for var in self.sv_dict.keys():
            dict_vars[var] = "self.sv_dict['{}']['var']".format(var)

        for var in self.fun_dict.keys():
            dict_vars[var] = "self.fun_dict['{}']['fun']".format(var)

        # these are all the state variable found in sv_dict and fun_dict
        all_variables = list(self.sv_dict.keys()) + list(self.fun_dict.keys())

        regex_vars = '\\b|'.join(sorted(re.escape(k) for k in all_variables))
        regex_math = '\\b|'.join(sorted(re.escape(k) for k in self.getValidOperators()['math']))

        # If repl is a function, it is called for every non-overlapping occurrence of pattern.
        modified_fun = re.sub(regex_vars, lambda m: dict_vars.get(m.group(0)), str_fun, flags=re.IGNORECASE)
        modified_fun = re.sub(regex_math, lambda m: 'math.{}'.format(m.group(0)), modified_fun, flags=re.IGNORECASE)

        # parse str to code
        try:
            res = parser.expr(modified_fun)
        except Exception as e:
            self.logger.warning('gui_receiver.py: {}'.format(e))
            return fun

        code = res.compile()

        # todo add try exception + logger

        try:
            fun = eval(code)
        except Exception as e:
            self.logger.warning('gui_receiver.py: {}'.format(e))

        used_variables = list()
        for var in self.sv_dict.values():
            if cs.depends_on(fun, var["var"]):
                used_variables.append(var["var"])


        return fun, used_variables

    def editFunction(self, name, str_fun):

        if name in self.fun_dict.keys():
            flag_syntax, signal_syntax = self._createAndAppendFun(name, str_fun)
            if flag_syntax:
                signal = 'Function "{}" edited with {}. Updated function: {}'.format(name, str_fun, self.fun_dict[name])
                return True, signal
            else:
                return False, signal_syntax
        else:
            signal = 'Failed editing of function "{}".'.format(name)
            return False, signal

    def updateFunctionNodes(self, name, nodes):
        print(nodes)
        self.fun_dict[name]['active'].setNodes(nodes, erasing=True)

    def updateFunctionUpperBounds(self, name, ub, nodes):
        self.fun_dict[name]['active'].setUpperBounds(ub, nodes)

    def updateFunctionLowerBounds(self, name, ub, nodes):
        self.fun_dict[name]['active'].setLowerBounds(ub, nodes)

    def updateFunctionBounds(self, name, lb, ub, nodes):
        self.fun_dict[name]['active'].setBounds(lb, ub, nodes)

    def getFunctionDict(self):
        return self.fun_dict

    def getFunction(self, name):
        if name in self.fun_dict.keys():
            return self.fun_dict[name]
        #todo change? return only active?

    def getVarDict(self):
        return self.sv_dict

    def getVar(self, elem):

        return self.sv_dict[elem]

    def getNodes(self):
        return self.nodes

    def setHorizonNodes(self, n_nodes):
        self.nodes = n_nodes
        self.casadi_prb.setNNodes(self.nodes)

    def _createAndAppendFun(self, name, str_fun):

        fun, used_vars = self.fromTxtToFun(str_fun)
        # fill horizon_receiver.fun_dict and funList

        if fun is not None:
            self.fun_dict[name] = dict(fun=fun, str=str_fun, active=None, used_vars=used_vars)
            return True
        else:
            return False

    def generate(self):
        try:
            self.casadi_prb.createProblem()
        except Exception as e:
            self.logger.warning('gui_receiver.py: {}'.format(e))
        return True

    def solve(self):
        try:
            self.casadi_prb.solveProblem()
        except Exception as e:
            self.logger.warning('gui_receiver.py: {}'.format(e))
        return True

    def getInfoAtNodes(self, node):
        vars = list()
        vars_dict = self.casadi_prb.scopeNodeVars(node) #lb, ub, w0
        if vars_dict is not None:
            for var in vars_dict.values():
                vars.append(var)

        cnstrs = self.casadi_prb.scopeNodeConstraints(node)
        costfuns = self.casadi_prb.scopeNodeCostFunctions(node)

        return vars, cnstrs, costfuns

    def plot(self):
        self.plt.plotVariables()

    def serialize(self):

        self.casadi_prb.serialize()


        # # serialize state variables
        # for name, data in self.sv_dict.items():
        #     self.logger.debug('Serializing variable "{}": {}'.format(name, data['var']))
        #     self.sv_dict[name]['var'] = data['var'].serialize()
        #
        # # serialize functions
        # print(self.casadi_prb.getConstraints())
        # print(self.fun_dict)
        #
        # for elem in self.fun_dict.values():
        #     if 'active' in elem:
        #         print(elem['active'])
        #         del elem['active']

        for name, data in self.fun_dict.items():
            if self.logger:
                self.logger.debug('Serializing function "{}": {}'.format(name, data['fun']))
            self.fun_dict[name]['fun'] = data['fun'].serialize()

    def deserialize(self):

        self.casadi_prb.deserialize()

        # # deserialize state variables
        # for name, data in self.sv_dict.items():
        #     self.sv_dict[name]['var'] = cs.SX.deserialize(data['var'])

        # for name, data in self.casadi_prb.getConstraints().items():
        #     print(name)
        #     print(data)
        #     # self.fun_dict[name]['fun'] =
        #     self.fun_dict[name]['active'] = data
        # deserialize functions
        for name, data in self.fun_dict.items():
            self.fun_dict[name]['fun'] = cs.SX.deserialize(data['fun'])

    def getValidOperators(self):
        '''
        return dictionary:
        keys: packages imported
        values: all the elements from the imported package that are considered "valid"
        '''

        full_list = dict()
        full_list['math'] = [elem for elem in dir(math) if not elem.startswith('_')]
        full_list['cs'] = ['cs.' + elem for elem in dir(cs) if not elem.startswith('_')]
        return full_list

if __name__ == '__main__':

    impl = horizonImpl(5)
    impl.addStateVariable(dict(name='x', dim=1, prev=0))
    impl.addStateVariable(dict(name='y', dim=1, prev=0))

    fun = dict(name='asd', str='x+y')
    impl.addFunction(fun)

    impl.serialize()
    pickle.dumps(impl)




