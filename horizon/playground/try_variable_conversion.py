from horizon.problem import Problem
import numpy as np

prb = Problem(3)

x = prb.createStateVariable('x', 1)
p = prb.createStateVariable('p', 1)

fun_cnsrt = np.sin(x) + p
one_cnsrt = prb.createConstraint('one_const', fun_cnsrt)

fun_cost = np.cos(x) + p
one_cost = prb.createCostFunction('one_cost', fun_cost)

print(f'converting {p} to parameter.')
par = prb.toParameter('p')

one_cnsrt_var_new = one_cnsrt.getVariables()
one_cost_var_new = one_cost.getVariables()

# for name, cnsrt in prb.getConstraints().items():
#     print(name)
#     for var in cnsrt.getVariables():
#         print(var, type(var))



# for name, var in prb.getVariables().items():
#     print(name, type(var))



# for elem in one_cnsrt.getVariables():
#     print(f'{elem.getName()}: {type(elem)}')





