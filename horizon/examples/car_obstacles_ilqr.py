#!/usr/bin/env python3

from horizon import problem
from horizon.solvers import ilqr
import casadi as cs
import numpy as np


def barrier(x):
    return 0.5 * (x + 0.001 * cs.sqrt((x * 1000) ** 2 + 1e-9))


N = 50  # number of nodes
dt = 0.1  # discretizaton step
niter = 50  # ilqr iterations

prb = problem.Problem(N-1)

# CREATE VARIABLES
x = prb.createStateVariable('x', 3)
u = prb.createInputVariable('u', 2)

xdot = cs.vertcat( u[0] * cs.cos(x[2]),
                   u[0] * cs.sin(x[2]),
                   u[1])

x0 = np.array([0, 0, 0]) # initial state
xf = np.array([0, 1, 0]) # desired final state

obs_center = np.array([0.05, 0.5])
obs_r = 0.1
obs = -cs.sumsqr(x[0:1] - obs_center) + obs_r

l = cs.sumsqr(u)  # intermediate cost
lf = 100.*cs.sumsqr(x - xf)  # final cost
gf = x - xf



solver = ilqr.iterativeLQR(x = x, u = u, xdot=xdot,
                           dt=dt, N=N,
                           intermediate_cost=l,
                           final_cost=lf,
                           final_constraint=None)

solver.setInitialState(x0)
np.random.seed(11311)
solver._use_single_shooting_state_update = True
# solver._use_second_order_dynamics = True
solver.randomizeInitialGuess()
solver.solve(10)


if True:

    import matplotlib.pyplot as plt

    plt.figure(figsize=[12, 5])
    xtrj = np.column_stack(solver._state_trj)
    lines = plt.plot(xtrj[0,:], xtrj[1,:], 's-')
    circle = plt.Circle(obs_center, radius=obs_r, fc='r')
    # plt.gca().add_patch(circle)
    plt.title('XY trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid()
    # plt.legend(lines, ['x', 'y', r'$\theta$'])


    # In[23]:
    plt.figure(figsize=[12, 5])
    plt.plot(solver._dcost, label='cost')
    plt.plot(solver._dx_norm, label='dx')
    plt.plot(solver._du_norm, label='du')
    plt.title('Increments')
    plt.xlabel('Iteration')
    plt.semilogy()
    plt.grid()
    plt.legend()

    plt.figure(figsize=[12, 5])
    lines = plt.plot(solver._state_trj)
    plt.title('State trajectory')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.grid()
    plt.legend(lines, ['x', 'y', r'$\theta$'])

    plt.figure(figsize=[12, 5])
    lines = plt.plot(solver._ctrl_trj)
    plt.title('Control trajectory')
    plt.xlabel('Time')
    plt.ylabel('Control')
    plt.grid()
    plt.legend(lines, ['v', r'$\dot{\theta}$'])

    plt.figure(figsize=[12, 5])
    lines = plt.plot(solver._defect)
    plt.title('Dynamics error')
    plt.xlabel('Time')
    plt.ylabel('State defect')
    plt.grid()
    plt.legend(lines, ['x', 'y', r'$\theta$'])

    plt.show()

print(solver._dcost)

cost_est = 0.0

for i in range(len(solver._ctrl_trj)):
    cost_est += dt* np.linalg.norm(solver._ctrl_trj[i])**2

Lf = cs.Function('final_cost',
                {'x': x, 'l': lf},
                ['x'],
                ['l']
                )

cost_est += Lf(x=solver._state_trj[-1])['l'].__float__()

