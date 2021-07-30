import sys
import os

import numpy as np
import casadi as cs
from horizon.problem import Problem as HorizonProblem

pyilqr_build_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build')
sys.path.append(pyilqr_build_folder)
import pyilqr

N = 10

pb = HorizonProblem(N)

x = pb.createStateVariable('x', 1)
u = pb.createInputVariable('u', 1)

fdyn = cs.Function('f', [x, u], [x + 0.1*u], ['x', 'u'], ['f'])

ilqr = pyilqr.IterativeLQR(fdyn, N)
x0 = np.array([1])
ilqr.setInitialState(x0)
for _ in range(10):
    ilqr.linearize_quadratize()
    ilqr.backward_pass()
    ilqr.forward_pass(1)

xtrj = ilqr.getStateTrajectory()

print(xtrj)