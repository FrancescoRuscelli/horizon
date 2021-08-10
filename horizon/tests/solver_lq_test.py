import unittest

from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.utils.transcription_methods import TranscriptionsHandler
import casadi as cs
import numpy as np
np.set_printoptions(suppress=True, precision=3)



# on a linear-quadratic problem, all solvers should agree on the solution
N = 5
dt = 0.1
prob = Problem(N)

# a random linear dynamics
x1 = prob.createStateVariable('x1', dim=2)
x2 = prob.createStateVariable('x2', dim=2)
x3 = prob.createStateVariable('x3', dim=2)
u1 = prob.createInputVariable('u1', dim=2)
u2 = prob.createInputVariable('u2', dim=2)
x = prob.getState().getVars()


A11 = cs.DM.rand(2, 2)
A13 = cs.DM.rand(2, 2)
A21 = cs.DM.rand(2, 2)
A32 = cs.DM.rand(2, 2)
B21 = cs.DM.rand(2, 2)
B32 = cs.DM.rand(2, 2)

xdot = cs.vertcat(
    A11@x1 + A13@x3,
    A21@x1 + B21@u1, 
    A32@x2 + B32@u2
)
prob.setDynamics(xdot)

# a random cost
prob.createIntermediateCost('c12', cs.sumsqr(x1 + x2))
prob.createIntermediateCost('c23', cs.sumsqr(x2 + x3))
prob.createIntermediateCost('c13', cs.sumsqr(x1 + x3))
prob.createIntermediateCost('u', cs.sumsqr(u1) + cs.sumsqr(u2))

# a final constraint
xtgt = np.array([1, 1, 2, 2, 3, 3])
prob.createFinalConstraint('xtgt', x - xtgt)

# an initial state
x0 = -xtgt
prob.getState().setBounds(lb=x0, ub=x0, nodes=0)

# solve first with ilqr
ilqrsol = Solver.make_solver('ilqr', prob, dt, 
        opts={'max_iter': 1, 'ilqr.integrator': 'EULER'})
ilqrsol.solve()

# solver with sqp
th = TranscriptionsHandler(prob, dt)
th.setDefaultIntegrator(type='EULER')
th.setMultipleShooting()
bsqpsol = Solver.make_solver('blocksqp', prob, dt, opts={'hess_update': 4})
bsqpsol.solve()

max_x_err = np.abs(ilqrsol.x_opt - bsqpsol.x_opt).max()
max_u_err = np.abs(ilqrsol.u_opt - bsqpsol.u_opt).max()
print(f'\n\nmax_x_err = {max_x_err} --- max_u_err = {max_u_err}')