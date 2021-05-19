from classes import problem as csprb

# def flatten(l):
#     if isinstance(l, list):
#         for e in l:
#             yield from flatten(e)
#     else:
#         yield l

if __name__ == '__main__':

    prb = csprb.Problem(10)

    x = prb.createStateVariable('x', 6)
    x_prev = prb.createStateVariable('x', 6, prev_nodes=-1) # how to do for previous nodes?
    x_prev2 = prb.createStateVariable('x', 6, prev_nodes=-2) # how to do for previous nodes?
    u = prb.createStateVariable('u', 4)
    z = prb.createStateVariable('z', 2)

    print(x)
    print(x_prev)
    print(x_prev2)
    print(u)
    print(z)

    # todo return a variable so that it can be used instead of class attributes
    # todo saving a variable (or a function) inside the problem is probably completely useless if it's not a STATE variable

    # todo how to check if a function has variable that are not defined in the problem? (they are not in state_var)

    # todo add logic for input variables and state variables
    # todo add bounds
    # start how to choose horizon? N? N-1?

    state_fun = x_prev[2:6] + u
    fun = u[2:4] + z **2

    # prb.setVariable('state_fun', state_fun) # is it ok?
    # prb.setVariable('fun', fun)

    cnsrt_x = prb.createConstraint('generic_constraint', x[0:2] - x[4:6], nodes=[[0, 2], [3, 4]])
    cnsrt_z = prb.createConstraint('yet_another_constraint', z - x[2:4], nodes=5)
    # cnsrt_x.setNodes(5)



    zmp_old = x_prev2[0:2] - x_prev2[4:6]  # * (h / grav)
    zmp = x[0:2] - x[4:6]  # * (h / grav)

    costfun_x = prb.createCostFunction('one_cost_function', zmp[0] - x[2])

    prb.createProblem()

    print('constraint name:', cnsrt_x.getName())
    print('constraint nodes:', cnsrt_x.getNodes())
    print('constraint function:', cnsrt_x.getFunction())
    print('constraint variables:', cnsrt_x.getVariables())
    print('----------------------------------------------')
    print('cost function name:', cnsrt_z.getName())
    print('cost function nodes:', cnsrt_z.getNodes())
    print('cost function function:', cnsrt_z.getFunction())
    print('cost function variables:', cnsrt_z.getVariables())
    print('----------------------------------------------')
    print('cost function name:', costfun_x.getName())
    print('cost function nodes:', costfun_x.getNodes())
    print('cost function function:', costfun_x.getFunction())
    print('cost function variables:', costfun_x.getVariables())



    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]], bounds=(dict(nodes=[0,2]))) # should be wrong
    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]], bounds=(dict(ubg=[1, 1])))