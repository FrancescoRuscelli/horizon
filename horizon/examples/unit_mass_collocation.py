#!/usr/bin/env python3

import horizon.problem as prb
import horizon.utils.plotter as plotter
import casadi as cs
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.solvers import solver
import matplotlib.pyplot as plt

def make_integrator(x, xdot, u, l):
    """A trivial Euler integrator"""
    dt = cs.SX.sym('dt', 1)
    xnext = x + xdot*dt
    lint = l*dt 
    return cs.Function('F_int', [x, u, dt], [xnext, lint], ['x0', 'p', 'dt'], ['xf', 'qf'])

N = 10
dt = 0.1
nx = 2
prob = prb.Problem(N=N)
use_transcription_methods = True

p = prob.createStateVariable('pos', dim=1)
v = prob.createStateVariable('vel', dim=1)
F = prob.createInputVariable('force', dim=1)

if use_transcription_methods:

    print('using utility "transcription_methods"')
    state = prob.getState()
    state_prev = state.getVarOffset(-1)
    x = cs.vertcat(state.getVars())


    xdot = cs.vertcat(v, F)
    prob.setDynamics(xdot)
    prob.setDt(dt)
    l = cs.sumsqr(F)  # useless

    use_ms = True
    if use_ms:  # multiple shooting
        my_integrator = make_integrator(x, xdot, F, l)
        th = Transcriptor.make_method('multiple_shooting', prob, opts=dict(integrator=my_integrator))
    else:
        th = Transcriptor.make_method('direct_collocation', prob)


else:

    p_prev = p.getVarOffset(-1)
    v_prev = v.getVarOffset(-1)
    F_prev = F.getVarOffset(-1)

    x = cs.vertcat(p, v)
    x_prev = cs.vertcat(p_prev, v_prev)
    xdot = cs.vertcat(v, F)
    prob.setDynamics(xdot)
    l = cs.sumsqr(F)  # useless


    my_integrator = make_integrator(x, xdot, F, l, dt)
    ms = prob.createConstraint('ms', my_integrator(x_prev, F_prev)[0] - x, nodes=range(1, N+1))


# set initial state (rest in zero)
p.setBounds(lb=0, ub=0, nodes=0)
v.setBounds(lb=0, ub=0, nodes=0)

# final constraint
p.setBounds(lb=1, ub=1, nodes=N)
v.setBounds(lb=0, ub=0, nodes=N)

# interediate cost
prob.createCost('cost', cs.sumsqr(F), nodes=range(N))  # TODO: intermediate vs final cost

# solve
opts={'ipopt.max_iter': 10}
solver = solver.Solver.make_solver('ipopt', prob, opts)
solver.solve()

solution = solver.getSolutionDict()

# plot
plot_all = True
if plot_all:
    hplt = plotter.PlotterHorizon(prob, solution)
    hplt.plotVariables()
    hplt.plotFunctions()
    plt.show()
