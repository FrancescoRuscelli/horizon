import sys
import os

import numpy as np
import casadi as cs
from horizon.problem import Problem as HorizonProblem
from horizon.utils import integrators

import matplotlib.pyplot as plt


np.set_printoptions(suppress=True, precision=3)

# pyilqr_build_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
# sys.path.append(pyilqr_build_folder)
# import pyilqr

from horizon.solvers import pyilqr

from matplotlib import pyplot as plt

# create problem
N = 100
dt = 0.03
pb = HorizonProblem(N)

# create variables
p = pb.createStateVariable('p', 2)
theta = pb.createStateVariable('theta', 1)
v = pb.createInputVariable('v', 1)
omega = pb.createInputVariable('omega', 1)

# define dynamics 
x = pb.getState().getVars()
u = pb.getInput().getVars()
xdot = cs.vertcat(v*cs.cos(theta), 
                  v*cs.sin(theta),
                  omega)

# discretize dynamics
xnext = integrators.RK4({'x': x, 'p': u, 'ode': xdot, 'quad': 0}, {'tf': dt})(x0=x, p=u)['xf']

# define functions to set to ilqr dynamics, cost, and constraints
fdyn = cs.Function('dyn', [x, u], [xnext], ['x', 'u'], ['f'])
l = cs.Function('l', [x, u], [1e-6*cs.sumsqr(u)], ['x', 'u'], ['l'])
x_tgt = np.array([1.0, 0.0, 0.0])
lf = cs.Function('lf', [x, u], [cs.sumsqr(x - x_tgt)], ['x', 'u'], ['l'])
cf = cs.Function('cf', [x, u], [x - x_tgt], ['x', 'u'], ['h'])

ilqr = pyilqr.IterativeLQR(fdyn, N)
ilqr.setIntermediateCost([l]*N)
ilqr.setFinalConstraint(cf)

# set initial state
x0 = np.array([0, 0, np.pi/2])
ilqr.setInitialState(x0)

# some stuff to interactively show each iteration 
plt.ion()
fig, axs = plt.subplots(1, 2)
xline = axs[0].plot(ilqr.getStateTrajectory().T, '-')
uline = axs[1].plot(ilqr.getInputTrajectory().T, '-')

for ax in axs: ax.grid()
axs[0].legend(['x', 'y', 'theta'])
axs[0].set_title('State trajectory')
axs[1].legend(['v', 'omega'])
axs[1].set_title('Input trajectory')

def iter_callback(xtrj, utrj, du, cost, defect, constr):
    fmt = ' <#010.3f'
    print(f'delta_u={du:{fmt}}  cost={cost:{fmt}}  constr={constr:{fmt}}  gap={defect:{fmt}}')
    for i, l in enumerate(xline):
        l.set_data(np.arange(N+1), xtrj[i, :])
    for i, l in enumerate(uline):
        l.set_data(np.arange(N), utrj[i, :])
    for ax in axs: 
        ax.relim()
        ax.autoscale_view()
    fig.canvas.draw_idle()
    plt.waitforbuttonpress()

ilqr.setIterationCallback(iter_callback)
print('****started!****')
print('press button to go to next iteration!')
plt.draw()

# solve!
ilqr.solve(10)
prof_info = ilqr.getProfilingInfo()

# print timings

print('\n\ntimings (inner):')
for k, v in prof_info.timings.items():
    if '_inner' not in k:
        continue
    print(f'{k[:-6]:30}{np.mean(v)} us')


print('\ntimings (iter):')

for k, v in prof_info.timings.items():
    if '_inner' in k:
        continue
    print(f'{k:30}{np.mean(v)} us')


# keep last plot alive
plt.ioff()
plt.show()