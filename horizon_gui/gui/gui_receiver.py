from horizon import problem as horizon
from horizon.solvers import Solver
from horizon_gui.gui.dynamics_module.dynamics_handler import DynamicsHandler
from horizon_gui.custom_functions.txt_to_fun import TxtToFun
from horizon.transcriptions.transcriptor import Transcriptor
import parser
import re
import casadi as cs
import pickle
from logging import DEBUG
from horizon.utils import plotter as plt

class horizonImpl():
    def __init__(self, nodes, logger=None):

        # #todo logger! use it everywhere!
        self.logger = logger

        self.nodes = nodes # this is without the last node!!!
        self.casadi_prb = horizon.Problem(self.nodes, logging_level=DEBUG)

        self.sv_dict = dict()  # state variables
        self.fun_dict = dict() # functions

        self.plt = plt.PlotterHorizon(self.casadi_prb)
        # self.active_fun_list = list()
        self.solver = None

        # todo hack!
        self.dt = 0.01

        # list of dynamics that can be defined
        self.txt_to_fun_converter = TxtToFun(self.sv_dict, self.fun_dict, self.logger)
        self.dyn_han = DynamicsHandler(self.casadi_prb, self.logger)

        self.trans_method = None
        self.dynamics_flag = False


    def _setVarGenerator(self, var_type):
        if var_type == 'State':
            return self.casadi_prb.createStateVariable
        if var_type == 'Input':
            return self.casadi_prb.createInputVariable
        if var_type == 'Single':
            return self.casadi_prb.createSingleVariable
        if var_type == 'Custom':
            return self.casadi_prb.createVariable

    def createVariable(self, var_type, name, dim, offset, nodes=None):

        flag, signal = self.checkVariable(name)

        if flag:

            if offset == 0:
                if var_type == 'State':

                    if self.dynamics_flag:
                        self.casadi_prb.resetDynamics()
                        self.dynamics_flag = False
                        self.logger.warning('system dynamics has been resetted due to the adding of a new state variable')
                    var = self.casadi_prb.createStateVariable(name, dim)
                if var_type == 'Input':
                    var = self.casadi_prb.createInputVariable(name, dim)
                if var_type == 'Single':
                    var = self.casadi_prb.createSingleVariable(name, dim)
                if var_type == 'Custom':
                    var = self.casadi_prb.createVariable(name, dim, nodes)
            else:
                raise Exception('TBD prev/next state variables')

            self.sv_dict[name] = dict(var=var, dim=dim, type=var_type)

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

    def activateFunction(self, name, fun_type, nodes):

        flag, signal = self.checkActiveFunction(name)

        if flag:
            if fun_type == 'constraint':
                try:
                    active_fun = self.casadi_prb.createConstraint(name, self.fun_dict[name]['fun'], nodes=nodes)
                    # self.active_fun_list.append(active_fun)
                    self.fun_dict[name].update({'active': active_fun})
                except Exception as e:
                    return False, e

            elif fun_type == 'cost':
                try:
                    active_fun = self.casadi_prb.createCostFunction(name, self.fun_dict[name]['fun'], nodes=nodes)
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
        elif active_fun_type == 'cost':
            self.casadi_prb.removeCostFunction(name)
        else:
            return False, 'Function type "{}" not recognized'.format(active_fun_type)

        return True, 'Function "{}" successfully removed.'.format(name)

    def removeStateVariable(self, name):
        try:
            self.casadi_prb.removeVariable(name)
            del self.sv_dict[name]
            if self.logger:
                self.logger.info(f'Variable "{name}" successfully removed.')
            return True
        except Exception as e:
            if self.logger:
                self.logger.warning(f'Failed to remove variable "{name}": {e}')
            return False

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
            flag = self._createAndAppendFun(name, str_fun)
            if flag:
                signal = 'Function "{}" edited with {}. Updated function: {}'.format(name, str_fun, self.fun_dict[name])
                return True, signal
            else:
                return False
        else:
            signal = 'Failed editing of function "{}".'.format(name)
            return False, signal

    def setTranscriptionMethod(self, type, opts):
        try:
            # remove old transcription methods if present

            trans_cnsrt = ['multiple_shooting', 'direct_collocation']
            for cnsrt in trans_cnsrt:
                if self.casadi_prb.getConstraints(cnsrt):
                    self.casadi_prb.removeConstraint(cnsrt)

            self.trans_method = dict(type=type, opts=opts)

            Transcriptor.make_method(type, self.casadi_prb, self.dt, opts=opts)
            self.transcription_flag = True
        except Exception as e:
            self.logger.warning('gui_receiver.py, setTranscriptionMethod: {}'.format(e))

    def getTranscriptionMethod(self):
        return self.trans_method

    def updateFunctionNodes(self, name, nodes):
        self.fun_dict[name]['active'].setNodes(nodes, erasing=True)

    def updateFunctionUpperBounds(self, name, ub, nodes):
        self.fun_dict[name]['active'].setUpperBounds(ub, nodes)

    def updateFunctionLowerBounds(self, name, lb, nodes):
        self.fun_dict[name]['active'].setLowerBounds(lb, nodes)

    def updateFunctionBounds(self, name, lb, ub, nodes):
        self.fun_dict[name]['active'].setBounds(lb, ub, nodes)

    def updateVarLb(self, name, lb, nodes):
        self.sv_dict[name]['var'].setLowerBounds(lb, nodes)

    def updateVarUb(self, name, ub, nodes):
        self.sv_dict[name]['var'].setUpperBounds(ub, nodes)

    def updateVarIg(self, name, ub, nodes):
        self.sv_dict[name]['var'].setInitialGuess(ub, nodes)

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

    def setDynamics(self, type, dyn):
        if type == 'default':
            self.dyn_han.set_default_dynamics(dyn)
            self.dynamics_flag = True
        elif type == 'custom':
            dyn, used_vars = self.txt_to_fun_converter.convert(dyn)
            self.dyn_han.set_custom_dynamics(dyn)
            self.dynamics_flag = True

    def isDynamicsReady(self):
        return self.dynamics_flag

    def getDefaultDynList(self):
        return self.dyn_han.getList()

    def _createAndAppendFun(self, name, str_fun):

        fun, used_vars = self.txt_to_fun_converter.convert(str_fun)
        # fill horizon_receiver.fun_dict and funList

        if fun is not None:
            self.fun_dict[name] = dict(fun=fun, str=str_fun, active=None, used_vars=used_vars)
            return True
        else:
            return False

    def generate(self):
        try:
            # if self.solver is None:
            #     self.logger.warning('Solver not set. Please select a valid solver before building Horizon problem.')
            # else:

            # todo add selection to choose solver
            self.solver = Solver.make_solver('ipopt', self.casadi_prb, self.dt)
            self.logger.info('Problem created successfully!')
        except Exception as e:
            self.logger.warning('gui_receiver.py: {}'.format(e))
            return False
        return True

    def solve(self):
        try:
            if self.solver is None:
                self.logger.warning('Solver not set. Cannot solve.')
            else:
                self.solver.solve()
                self.logger.info('Problem solved succesfully!')
        except Exception as e:
            self.logger.warning('gui_receiver.py: {}'.format(e))
            return False
        return True

    def getInfoAtNodes(self, node):
        raise Exception('getInfoAtNodes yet to be done')
        vars = list()
        vars_dict = self.casadi_prb.scopeNodeVars(node) #lb, ub, w0
        if vars_dict is not None:
            for var in vars_dict.values():
                vars.append(var)

        cnstrs = self.casadi_prb.scopeNodeConstraints(node)
        costfuns = self.casadi_prb.scopeNodeCostFunctions(node)

        return vars, cnstrs, costfuns

    def plot(self):
        self.plt.setSolution(self.solver.getSolutionDict())
        self.plt.plotVariables()
        self.plt.plotFunctions()

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

if __name__ == '__main__':

    impl = horizonImpl(5)
    impl.addStateVariable(dict(name='x', dim=1, prev=0))
    impl.addStateVariable(dict(name='y', dim=1, prev=0))

    fun = dict(name='asd', str='x+y')
    impl.addFunction(fun)

    impl.serialize()
    pickle.dumps(impl)




