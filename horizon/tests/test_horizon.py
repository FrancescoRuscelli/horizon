from horizon.problem import Problem
import pprint
import numpy as np
import logging

def test_singleParameter():
    nodes = 10

    prb = Problem(nodes, crash_if_suboptimal=True)
    x = prb.createStateVariable('x', 6)
    u = prb.createInputVariable('u', 2)
    p = prb.createSingleParameter('p', 6)

    x.setBounds([1, 1, 1, 1, 1, 1], [2, 2, 2, 2, 2, 2])

    constr1 = prb.createIntermediateConstraint('constr', x[2:4] ** 2 + p[2:4] - u)

    constr1.setBounds([1, 1], [1, 1])
    prb.createProblem()

    all_sol = dict()
    for i in range(100):
        p.assign(6 * [2*i])
        sol = prb.solveProblem()
        all_sol[i] = sol

    pprint.pprint(all_sol)

def test_parameters():
    #  check if bounds are correct when setting them AFTER building the problem
    nodes = 10

    prb = Problem(nodes, crash_if_suboptimal=True)
    x = prb.createStateVariable('x', 6)
    u = prb.createInputVariable('u', 2)
    p = prb.createParameter('p', 1)

    constr1 = prb.createIntermediateConstraint('constr', x[2] ** 2 + p - u[1])

    constr1.setBounds(1, 1)

    prb.createProblem()

    x.setBounds([1, 1, 1, 1, 1, 1], [2, 2, 2, 2, 2, 2])

    cos_fun = np.cos(np.linspace(0, 2*np.pi, 11))

    for n in range(nodes+1):
        p.assign(cos_fun[n], n)

    sol = prb.solveProblem()

def test_singleVariable():
    #  check if bounds are correct when setting them AFTER building the problem
    nodes = 10

    prb = Problem(nodes, crash_if_suboptimal=True)
    x = prb.createStateVariable('x', 6)
    u = prb.createSingleVariable('u', 2)

    constr1 = prb.createIntermediateConstraint('constr', x[2] ** 2 + u[1:3])

    constr1.setBounds(1, 1)

    prb.createProblem()

    x.setBounds([1, 1, 1, 1, 1, 1], [2, 2, 2, 2, 2, 2])

    cos_fun = np.cos(np.linspace(0, 2*np.pi, 11))

    sol = prb.solveProblem()

def test_constraintBounds():
    #  check if bounds are correct when setting them AFTER building the problem
    nodes = 10

    prb = Problem(nodes, crash_if_suboptimal=True)
    x = prb.createStateVariable('x', 2)
    u = prb.createSingleVariable('u', 2)

    constr1 = prb.createIntermediateConstraint('constr1', x+u, bounds=dict(lb=[0, 0]))
    constr2 = prb.createIntermediateConstraint('constr2', x + u, bounds=dict(ub=[0, 0]))
    constr3 = prb.createIntermediateConstraint('constr3', x + u, bounds=dict(lb=[0, 0], ub=[0, 0]))

    prb.createProblem()
    sol = prb.solveProblem()

if __name__ == '__main__':
    # test_singleParameter()
    # test_parameters()
    # test_singleVariable()
    test_constraintBounds()
    exit()
nodes = 10
prb = Problem(nodes, logging_level=logging.DEBUG)
x = prb.createInputVariable('x', 2)
diosporco = prb.createIntermediateConstraint('diosporcomaledetto', x)

prb.createProblem()
prb.solveProblem()
exit()

nodes = 10
prb = Problem(nodes, logging_level=logging.DEBUG)
x = prb.createStateVariable('x', 2)
v = prb.createStateVariable('v', 2)
k = prb.createVariable('k', 2, range(2, 8))
u = prb.createInputVariable('u', 2)
t_tot = prb.createVariable('t', 2)
p = prb.createSingleVariable('p', 2)
t_tot.setBounds([100, 100], [100, 100])

prb.setNNodes(5)
exit()
import horizon.utils.transcription_methods as tm
import horizon.utils.integrators as integ

nodes = 10
prb = Problem(nodes, logging_level=logging.DEBUG)
x = prb.createStateVariable('x', 2)
v = prb.createStateVariable('v', 2)
k = prb.createVariable('k', 2, range(2, 8))
u = prb.createInputVariable('u', 2)
t_tot = prb.createVariable('t', 2)
t_tot.setBounds([100, 100], [100, 100])

# danieli = prb.createConstraint('danieli', x-k, nodes=range(2, 8))
diosporco = prb.createConstraint('diosporcomaledetto', x - t_tot)
diosporco.setBounds([100, 100], [100, 100])
xprev = x.getVarOffset(-1)

xprev_copy = x.getVarOffset(-1)
xnext = x.getVarOffset(+1)

opts = {'ipopt.tol': 1e-4,
        'ipopt.max_iter': 2000}

prb.createProblem(opts=opts)

print(prb.getProblem()['x'])

prb.solveProblem()

exit()
# print(id(xprev))
# print(id(xprev_copy))
# exit()
state = prb.getState()
state_prev = state.getVarOffset(-1)

state.setBounds([0, 0], [0, 0], 2)
print(state.getBounds(2))
exit()

dt = 0.01
state_dot = cs.vertcat(v, u)
# opts = dict()
# opts['tf'] = dt
# dae = dict()
# dae['x'] = cs.vertcat(*prb.getState())
# dae['p'] = cs.vertcat(*prb.getInput())
# dae['ode'] = state_dot
# dae['quad'] = cs.sumsqr(u)

# integrator = integ.RK4(dae, opts, cs.SX)
hl = tm.TranscriptionsHandler(prb, 0.01, state_dot=state_dot)
# hl.set_integrator(integrator)
hl.setMultipleShooting()

prb.createProblem({'nlpsol.ipopt': 10})

prb.solveProblem()

exit()
# ==================================================================================================================
# ======================================= bounds as list but also other stuff =====================================
# ==================================================================================================================

nodes = 10
prb = Problem(nodes, logging_level=logging.DEBUG)
x = prb.createStateVariable('x', 2)
y = prb.createStateVariable('y', 2)
z = prb.createInputVariable('z', 2)
danieli = prb.createConstraint('danieli', x + y, range(4, 10))

x.setBounds([2, 2], [2, 2])
danieli.setBounds([12, 12], [12, 12], list(range(4, 10)))
prb.createProblem({"nlpsol.ipopt": True})
sol = prb.solveProblem()

print(sol)
exit()

# ==================================================================================================================
# ==================================================================================================================
# ==================================================================================================================

# nodes = 8
# prb = Problem(nodes)
# x = prb.createStateVariable('x', 2)
# y = prb.createStateVariable('y', 2)
# danieli = prb.createConstraint('danieli', x+y)
#
# danieli.setBounds([12, 12],[12, 12], 4)
# prb.createProblem()
# sol = prb.solveProblem()
#
# print(sol)
# exit()
#
#
# print('===PICKLING===')
# prb = prb.serialize()
# prb_serialized = pickle.dumps(prb)
#
# print('===DEPICKLING===')
# prb_new = pickle.loads(prb_serialized)
# prb_new.deserialize()
#
# sv_new = prb_new.getStateVariables()
# cnstr_new = prb_new.getConstraints()
#
# # these two are different
# print('x', x)
# print('new x', sv_new['x'])
#
# # todo how to check if the new state variable x is used by all the constraints?
#
# # oebus = prb_new.createConstraint('oebus', x)  # should not work
# oebus = prb_new.createConstraint('oebus', sv_new['x'])  # should work
# prb_new.createProblem()
#
# exit()
# ==================================================================================================================
# ==================================================================================================================
# ==================================================================================================================
# nodes = 8
# prb = Problem(nodes)
# x = prb.createStateVariable('x', 2)
# y = prb.createStateVariable('y', 2)
# # todo something wrong here
# danieli = prb.createConstraint('danieli', x+y)
# sucua = prb.createCostFunction('sucua', x*y)
#
#
# prb.createProblem()
#
# print('===PICKLING===')
#
# prb = prb.serialize()
# prb_serialized = pickle.dumps(prb)
#
#
# print('===DEPICKLING===')
# prb_new = pickle.loads(prb_serialized)
# prb_new.deserialize()
#
# prb_new.createProblem()
# print(prb_new.prob)
#
# exit()
# ==================================================================================================================
# ==================================================================================================================
# ==================================================================================================================

nodes = 8
prb = Problem(nodes, logging_level=logging.INFO)
x = prb.createStateVariable('x', 2)
y = prb.createStateVariable('y', 2)

x.setBounds([-2, -2], [2, 2])

# todo this is allright but I have to remember that if I update the nodes (from 3 to 6 for example) i'm not updating the constraint nodes
# todo so if it was active on all the node before, then it will be active only on the node 1, 2, 3 (not on 4, 5, 6)


scoping_node = nodes
# print('var at nodes {}  BEFORE creating the problem: '.format(scoping_node), prb.scopeNodeVars(scoping_node))
# print('number of nodes of {}: {}'.format(x, x.getNNodes()))
# print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))

# print('getVarImplList way before:', prb.state_var_container.getVarImplList())
danieli = prb.createConstraint('danieli', x + y)
sucua = prb.createCostFunction('sucua', x * y, nodes=list(range(3, 15)))
pellico = prb.createCostFunction('pellico', x - y, nodes=[0, 4, 6])

danieli.setBounds(lb=[-1, -1], ub=[1, 1], nodes=3)

prb.createProblem({"nlpsol.ipopt": True})

for i in range(nodes + 1):
    print(x.getBounds(i))

# print('var at nodes {}  AFTER creating the problem: '.format(scoping_node), prb.scopeNodeVars(scoping_node))
# print('getVarImplList before:', prb.state_var_container.getVarImplList())
new_n_nodes = 5
# print('================== Changing n. of nodes to {} =================='.format(new_n_nodes))
prb.setNNodes(new_n_nodes)
scoping_node = 8
# print('var at nodes {} AFTER changing the n. of nodes but BEFORE rebuilding: {}'.format(scoping_node, prb.scopeNodeVars(scoping_node)))
# print('number of nodes of {}: {}'.format(x, x.getNNodes()))
# print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))
# print('getVarImplList after but before create:', prb.state_var_container.getVarImplList())
prb.createProblem()

# todo check why this is so
# print('after:', prb.state_var_container.getVarImplList())

scoping_node = 8
# print('var at nodes {} AFTER changing the n. of nodes but AFTER rebuilding: {}'.format(scoping_node, prb.scopeNodeVars(scoping_node))) # should not work
# print('number of nodes of {}: {}'.format(x, x.getNNodes()))
# print('bounds of function {} at node {} are: {}'.format(x, scoping_node, x.getBounds(scoping_node)))
# x.setBounds(10)
# danieli.setNodes([1,6])
prb.scopeNodeVars(2)

x.setBounds([2, 8], [2, 8], 5)

for i in range(new_n_nodes + 1):
    print(x.getBounds(i))

# todo what do I do?
# is it better to project the abstract variable as soon as it is created, to set the bounds and everything?
# or is it better to wait for the buildProblem to generate the projection of the abstract value along the horizon line?
