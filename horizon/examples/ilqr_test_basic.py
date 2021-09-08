import sys
import os

import numpy as np
import casadi as cs
from horizon.problem import Problem as HorizonProblem

pyilqr_build_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
sys.path.append(pyilqr_build_folder)
import pyilqr

from matplotlib import pyplot as plt

N = 2

pb = HorizonProblem(N)

x = pb.createStateVariable('x', 1)
u = pb.createInputVariable('u', 1)

fdyn = cs.Function('dyn', [x, u], [x + 0.1*u], ['x', 'u'], ['f'])
l = cs.Function('l', [x, u], [1e-6*cs.sumsqr(u)], ['x', 'u'], ['l'])
lf = cs.Function('lf', [x, u], [cs.sumsqr(x - 1)], ['x', 'u'], ['l'])
cf = cs.Function('cf', [x, u], [x - 2], ['x', 'u'], ['h'])
c = cs.Function('c', [x, u], [x + 2], ['x', 'u'], ['h'])


ilqr = pyilqr.IterativeLQR(fdyn, N)
ilqr.setIntermediateCost([l]*N)
ilqr.setFinalConstraint(cf)
ilqr.setIntermediateConstraint(1, c)
x0 = np.array([3])
ilqr.setInitialState(x0)
ilqr.solve(1)

xtrj = ilqr.getStateTrajectory()
print(xtrj)
plt.plot(xtrj.T)
plt.title('State trajectory')
plt.show()