import re

# \([^()]*\)
# def partial_match(regex, string, flags=0, op=re.match):
#     """
#     Matches a regular expression to a string incrementally, retaining the
#     best substring matched.
#     :param regex:   The regular expression to apply to the string.
#     :param string:  The target string.
#     :param flags:   re module flags, e.g.: `re.I`
#     :param op:      Either of re.match (default) or re.search.
#     :return:        The substring of the best partial match.
#     """
#     # if regex[-1] == '|':
#     #     regex = regex[:-1]
#
#     # split all formula
#     # re.sub("[\(].*?[\)]", "", x)
#
#     m = op(regex, string)
#     if m:
#         return m.group(0)
#     final = None
#
#     for i in range(1, len(regex) + 1):
#         try:
#             if regex[:i][-1] != '|' and regex[:i][-1] != '\\':
#                 m = op(regex[:i], string, flags)
#                 # print('iter {}: try {} using {}. SOLUTION: {}'.format(i, regex[:i], string, m))
#             if m:
#                 final = m.group(0)
#         except re.error:
#             print('ERROR at iter {}: {}'.format(i, regex[:i]))
#             pass
#
#     return final



# culo = '((x|y|culo)(\[\d+:\d+\])?)+'

# culo = '(sqrt|cumsum|abs)\(((x|y|culo)(\[\d+:\d+\])?)\))+'
#
# # culo = '(\((ciao)'
#
# operators = ['sqrt', 'cumsum', 'abs']
#
# # split regex if literal  \( or \) are found
# result = re.split('\\\\\\(|\\\\\\)', culo)
#
#
# if result:
#     print(result)
#     for regex_portion in result:
#         if regex_portion == '':
#             # print('starting with ()')
#             continue
#         if regex_portion == '?':
#             # print('ignoring "?"')
#             continue
#         if regex_portion == '+':
#             # print('ignoring "+')
#             continue
#         if regex_portion in
#         partial_final = partial_match(regex_portion, 'd')

# import ast
# import operator as op
#
# # supported operators
# operators = {ast.Add: op.add, ast.Sub: op.sub, ast.Mult: op.mul,
#              ast.Div: op.truediv, ast.Pow: op.pow, ast.BitXor: op.xor,
#              ast.USub: op.neg}
#
# def eval_expr(expr):
#     return eval_(ast.parse(expr, mode='eval').body)
#
# def eval_(node):
#     if isinstance(node, ast.Num): # <number>
#         return node.n
#     elif isinstance(node, ast.BinOp): # <left> <operator> <right>
#         return operators[type(node.op)](eval_(node.left), eval_(node.right))
#     elif isinstance(node, ast.UnaryOp): # <operator> <operand> e.g., -1
#         return operators[type(node.op)](eval_(node.operand))
#     else:
#         raise TypeError(node)
#
#
# hello = eval_expr('sin(1*2)')
# print(hello)
import parser
import numpy as np
import casadi as cs
str_fun = 'xy + y[1:2] + y[1:2] + x + x[1:2] + xy'
sv_dict = {'x':'self.x', 'xy':'self.xy', 'z':'self.z', 'y':'self.y'}
regex_vars = '\\b|'.join(sorted(re.escape(k) for k in sv_dict))
# regex_vars = 'x\\b|y\\b|xy\\b|z\\b'
print(regex_vars)
str_fun = re.sub(regex_vars, lambda m: sv_dict.get(m.group(0)), str_fun, flags=re.IGNORECASE)

print(str_fun)


    # """
    #     >>> eval_expr('2^6')
    # 4
    # >>> eval_expr('2**6')
    # 64
    # >>> eval_expr('1 + 2*3**(4^5) / (6 + -7)')
    # -5.0
    # """