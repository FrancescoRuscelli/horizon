from classes import problem as horizon
import parser
import re

class horizonImpl():
    def __init__(self, nodes, logger=None):

        # #todo logger! use it everywhere!
        self.logger = logger

        self.nodes = nodes
        self.casadi_prb = horizon.Problem(self.nodes)

        self.sv_dict = dict()  # state variables
        self.fun_dict = dict() # functions

        self.active_fun_dict = dict()

    def addStateVariable(self, data):

        name = data['name']
        dim = data['dim']
        prev = data['prev']

        flag, signal = self.checkStateVariable(name)
        if flag:
            if prev == 0:
                var = self.casadi_prb.createStateVariable(name, dim)
            else:
                var = self.casadi_prb.createStateVariable(name, dim, prev)

            self.sv_dict[name] = dict(var=var, dim=dim)

            return True, signal
        else:
            return False, signal

    def addFunction(self, data):

        # get Function from Text
        name = data['name']
        str_fun = data['str']
        fun_type = data['type']

        flag, signal = self.checkFunction(name, str_fun)
        if flag:

            self._createAndAppendFun(name, str_fun, fun_type)
            return True, signal
        else:
            return False, signal

    def activateFunction(self, name, fun_type):

        if fun_type == 'constraint':
            try:
                # self.logger.info('Adding function: {}'.format(self.fun_dict[name]))
                self.casadi_prb.createConstraint(name, self.fun_dict[name]['fun'])
            except Exception as e:
                return False, e

        elif fun_type == 'costfunction':
            try:
                self.casadi_prb.createCostFunction(name, self.fun_dict[name]['fun'])
            except Exception as e:
                return False, e

        return True, 'Function "{}: {}" activated.'.format(name, self.fun_dict[name]['str'])

    def removeFunction(self, data):
        print('"removeFunction" yet to implement. Data: {}'.format(data))

    def removeStateVariable(self, data):
        print('"removeStateVariable" yet to implement. Data: {}'.format(data))

    def solveProblem(self):
        print('"solveProblem" yet to implement')

    def checkActiveFunction(self, name): # fun, name

        if name in self.active_fun_dict.keys():
            signal = "active function already inserted"
            return False, signal

        return True, 'Function "{}" can be activated. Adding.'

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

        return True, 'Function "{}" is acceptable. Adding'.format(name)

    def checkStateVariable(self, name):

        if name == "":
            signal = "State Variable: Empty Value Not Allowed"
            return False, signal
        elif name in self.sv_dict.keys():
            signal = "State Variable: Already Inserted"
            return False, signal
        elif name.isnumeric():
            signal = "State Variable: Invalid Name"
            return False, signal

        return True, "State Variable: generated '{}'".format(name)


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

        # If repl is a function, it is called for every non-overlapping occurrence of pattern.
        modified_fun = re.sub(regex_vars, lambda m: dict_vars.get(m.group(0)), str_fun, flags=re.IGNORECASE)

        # parse str to code
        res = parser.expr(modified_fun)
        code = res.compile()

        # todo add try exception + logger

        try:
            fun = eval(code)
        except Exception as e:
            self.logger.warning(e)

        return fun

    def editFunction(self, name, str_fun):

        fun = self.fromTxtToFun(str_fun)

        if name in self.fun_dict.keys():
            self._createAndAppendFun(name, str_fun, 'generic')
            signal = 'Function "{}" edited with {}. Updated function: {}'.format(name, str_fun, self.fun_dict[name])
            return True, signal
        else:
            signal = 'Failed editing of function "{}".'.format(name)
            return False, signal

    def getFunction(self, name):
        if name in self.fun_dict.keys():
            return self.fun_dict[name]

    def _createAndAppendFun(self, name, str_fun, fun_type):

        fun = self.fromTxtToFun(str_fun)
        # TODO I can probably do a wrapper function in casadi self.createFunction(name, fun, type)
        # TODO HOW ABOUT GENERIC FUNCTION? Should i do a casadi function for them?
        # fill horizon_receiver.fun_dict and funList
        # TODO add fun type??
        self.fun_dict[name] = dict(fun=fun, str=str_fun, type=fun_type)

        # todo what to do with fun of fun

        # if constraint has also a type, this:
        if fun_type == 'constraint':
            self.addConstraintFunction(name, fun)
        elif fun_type == 'costfunction':
            self.addCostFunction(name, fun)





