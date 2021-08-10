import sys
import os

import numpy as np
import casadi as cs
from horizon.problem import Problem as HorizonProblem

np.set_printoptions(suppress=True, precision=3)

pyilqr_build_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
sys.path.append(pyilqr_build_folder)
import pyilqr

from matplotlib import pyplot as plt

N = 20

pb = HorizonProblem(N)

q = pb.createStateVariable('q', 1)
qdot = pb.createStateVariable('qdot', 1)
u = pb.createInputVariable('u', 1)
x = pb.getState().getVars()
dt = 0.1
xnext = cs.vertcat(q + qdot*dt + 0.5*u*dt**2, 
                   qdot + dt*u)

fdyn = cs.Function('dyn', [x, u], [xnext], ['x', 'u'], ['f'])
l = cs.Function('l', [x, u], [1e-6*cs.sumsqr(u)], ['x', 'u'], ['l'])
x_tgt = np.array([1.0, 0.0])
lf = cs.Function('lf', [x, u], [cs.sumsqr(x - x_tgt)], ['x', 'u'], ['l'])
cf = cs.Function('cf', [x, u], [x - x_tgt], ['x', 'u'], ['h'])
c = cs.Function('c', [x, u], [qdot], ['x', 'u'], ['h'])


ilqr = pyilqr.IterativeLQR(fdyn, N)
# ilqr.setIntermediateCost([l]*N)
# ilqr.setFinalCost(lf)
ilqr.setFinalConstraint(cf)
ilqr.setIntermediateConstraint(N//2, c)
x0 = np.array([0, 0])
ilqr.setInitialState(x0)
ilqr.solve(10)

xtrj = ilqr.getStateTrajectory()
print(xtrj)
plt.plot(xtrj.T, 'd-')
plt.title('State trajectory')
plt.show()