#!/usr/bin/env python3

import horizon.problem as prb
import horizon.utils.plotter as plotter
import casadi as cs 
import numpy as np
from horizon.utils.integrators import make_direct_collocation

def make_integrator(x, xdot, u, l, dt):
    """A trivial Euler integrator"""
    xnext = x + xdot*dt
    lint = l*dt 
    return cs.Function('F_int', [x, u], [xnext, lint], ['x0', 'p'], ['xf', 'qf'])

N = 10
dt = 0.1
nx = 2
prob = prb.Problem(N=N)

p = prob.createStateVariable('pos', dim=1)
v = prob.createStateVariable('vel', dim=1)
F = prob.createInputVariable('force', dim=1)
p_prev = prob.createStateVariable('pos', dim=1, prev_nodes=-1)
v_prev = prob.createStateVariable('vel', dim=1, prev_nodes=-1)
F_prev = prob.createInputVariable('force', dim=1, prev_nodes=-1)

x = cs.vertcat(p, v)
x_prev = cs.vertcat(p_prev, v_prev)
xdot = cs.vertcat(v, F)
l = cs.sumsqr(F)  # useless


use_ms = False
if use_ms:  # multiple shooting
    my_integrator = make_integrator(x, xdot, F, l, dt)
    ms = prob.createConstraint('ms', my_integrator(x_prev, F_prev)[0] - x, nodes=range(1, N+1))
    ms.setBounds(lb=np.zeros(nx), ub=np.zeros(nx))
else:  # collocation
    make_direct_collocation(prob=prob, x=x, x_prev=x_prev, xdot=xdot, degree=3, dt=dt)

# set initial state (rest in zero)
p.setBounds(lb=0, ub=0, nodes=0)
v.setBounds(lb=0, ub=0, nodes=0)

# final constraint
p.setBounds(lb=1, ub=1, nodes=N)
v.setBounds(lb=0, ub=0, nodes=N)

# interediate cost
prob.createCostFunction('cost', cs.sumsqr(F), nodes=range(N))  # TODO: intermediate vs final cost

# solve
prob.createProblem(opts={'nlpsol.ipopt': 10})
solution = prob.solveProblem()

# plot
plt = plotter.PlotterHorizon(sol=solution)
plt.plotVariables()