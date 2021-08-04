import sys
import os

import numpy as np
import casadi as cs
from horizon.problem import Problem as HorizonProblem
from horizon.utils import integrators

np.set_printoptions(suppress=True, precision=3)

pyilqr_build_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
sys.path.append(pyilqr_build_folder)
import pyilqr

from matplotlib import pyplot as plt

N = 20
dt = 0.1

pb = HorizonProblem(N)

p = pb.createStateVariable('p', 2)
theta = pb.createStateVariable('theta', 1)
v = pb.createInputVariable('v', 1)
omega = pb.createInputVariable('omega', 1)

x = pb.getState().getVars()
u = pb.getInput().getVars()
xdot = cs.vertcat(v*cs.cos(theta), 
                  v*cs.sin(theta),
                  omega)

xnext = integrators.EULER({'x': x, 'p': u, 'ode': xdot, 'quad': 0}, {'tf': dt})(x0=x, p=u)['xf']

fdyn = cs.Function('dyn', [x, u], [xnext], ['x', 'u'], ['f'])
l = cs.Function('l', [x, u], [1e-6*cs.sumsqr(u)], ['x', 'u'], ['l'])
x_tgt = np.array([1.0, 0.0, 0.0])
lf = cs.Function('lf', [x, u], [cs.sumsqr(x - x_tgt)], ['x', 'u'], ['l'])
cf = cs.Function('cf', [x, u], [x - x_tgt], ['x', 'u'], ['h'])


ilqr = pyilqr.IterativeLQR(fdyn, N)
ilqr.setIntermediateCost([l]*N)
ilqr.setFinalConstraint(cf)
x0 = np.array([0, 0, np.pi/2])
ilqr.setInitialState(x0)
ilqr.solve(10)

xtrj = ilqr.getStateTrajectory()
utrj = ilqr.getInputTrajectory()

plt.figure()
plt.plot(xtrj.T, 'd-')
plt.title('State trajectory')

plt.figure()
plt.plot(utrj.T, 'd-')
plt.title('Input trajectory')

plt.show()