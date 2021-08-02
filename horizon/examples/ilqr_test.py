import sys
import os
from casadi.casadi import sumsqr

import numpy as np
import casadi as cs
from horizon.problem import Problem as HorizonProblem

pyilqr_build_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
sys.path.append(pyilqr_build_folder)
import pyilqr

from matplotlib import pyplot as plt

N = 20

pb = HorizonProblem(N)

x = pb.createStateVariable('x', 2)
u = pb.createInputVariable('u', 2)

fdyn = cs.Function('f', [x, u], [x + 0.1*u], ['x', 'u'], ['f'])
l = cs.Function('f', [x, u], [0.1*cs.sumsqr(u)], ['x', 'u'], ['l'])
lf = cs.Function('f', [x, u], [1000*(cs.sumsqr(x) - 4)**2], ['x', 'u'], ['l'])

ilqr = pyilqr.IterativeLQR(fdyn, N)
ilqr.setIntermediateCost([l]*N)
ilqr.setFinalCost(lf)
x0 = np.array([1, 1])
ilqr.setInitialState(x0)
for _ in range(10):
    ilqr.linearize_quadratize()
    ilqr.backward_pass()
    ilqr.forward_pass(1)

xtrj = ilqr.getStateTrajectory()

plt.plot(xtrj[0, :], xtrj[1, :])
plt.title('State trajectory')
plt.show()