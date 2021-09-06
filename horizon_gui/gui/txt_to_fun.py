import math
import casadi as cs
import parser
import re

class TxtToFun:
    def __init__(self, sv_dict, fun_dict, logger=None):

        self.sv_dict = sv_dict
        self.fun_dict = fun_dict

        self.logger = logger


    def convert(self, str_fun):

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

    @staticmethod
    def getValidOperators():
        '''
        return dictionary:
        keys: packages imported
        values: all the elements from the imported package that are considered "valid"
        '''

        full_list = dict()
        full_list['math'] = [elem for elem in dir(math) if not elem.startswith('_')]
        full_list['cs'] = ['cs.' + elem for elem in dir(cs) if not elem.startswith('_')]
        return full_list