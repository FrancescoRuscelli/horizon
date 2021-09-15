# todo somehow find a way to import the necessary stuff from GUI
import math
import casadi as cs
import numpy as np
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
            dict_vars[var] = f"self.sv_dict['{var}']['var']"

        for fun in self.fun_dict.keys():
            dict_vars[fun] = f"self.fun_dict['{fun}']['fun']"

        # these are all the state variable found in sv_dict and fun_dict
        all_variables = list(self.sv_dict.keys()) + list(self.fun_dict.keys())

        math_operators = self.getValidOperators()['math']

        # if a variable has the same name of an operator, var wins
        for overrided_var in [var for var in all_variables if var in math_operators]:
            math_operators.remove(overrided_var)

        # generate regex with all the variables found: x\b|y\b|z\b
        regex_vars = '\\b|'.join(sorted(re.escape(k) for k in all_variables))
        regex_math = '\\b|'.join(sorted(f'\b{re.escape(k)}\b' for k in math_operators))


        # If repl is a function, it is called for every non-overlapping occurrence of pattern.
        modified_fun = re.sub(regex_vars, lambda m: dict_vars.get(m.group(0)), str_fun, flags=re.IGNORECASE)
        modified_fun = re.sub(regex_math, lambda m: 'math.{}'.format(m.group(0)), modified_fun, flags=re.IGNORECASE)

        # parse str to code
        try:
            res = parser.expr(modified_fun)
        except Exception as e:
            self.logger.warning('gui_receiver.py: {}'.format(e))
            return None

        code = res.compile()

        # todo add try exception + logger

        try:
            fun = eval(code)
        except Exception as e:
            if self.logger:
                self.logger.warning('gui_receiver.py: {}'.format(e))

        return fun

    @staticmethod
    def getValidOperators():

        full_list = dict()
        full_list['math'] = [elem for elem in dir(math) if not elem.startswith('_')]
        full_list['cs'] = ['cs.' + elem for elem in dir(cs) if not elem.startswith('_')]
        return full_list

