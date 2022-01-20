#!/usr/bin/env python

import logging
import casadi as cs
from horizon import problem
from horizon.transcriptions import integrators
from horizon.solvers import solver
import matplotlib.pyplot as plt
import numpy as np
import time


N = 21  # Control discretization
T = 10.0  # End time
dt = T/N
prb = problem.Problem(N, logging_level=logging.DEBUG)

dx = prb.createStateVariable('dx', 2)
dx.setBounds([0., 1.], [0., 1.], nodes=[0])
dx.setBounds([0., 0.], [0., 0.], nodes=[20])
dx.setInitialGuess([0., 0.])


du = prb.createInputVariable('du', 1)
du.setBounds([-1.], [1.])
du.setInitialGuess([0.])


# FORMULATE DISCRETE TIME DYNAMICS
# System dynamics
xdot = cs.vertcat(*[(1. - dx[1] ** 2) * dx[0] - dx[1] + du, dx[0]])
prb.setDynamics(xdot)
prb.setDt(dt)

print(f'xdot: {xdot}')
dae = {'x': dx, 'p': du, 'ode': xdot, 'quad': []}
F_integrator = integrators.RK4(dae)
print(f"F_integrator: {F_integrator}")

# Cost function
prb.createCost('min_v', cs.vertcat(dx, du), nodes=list(range(0, N)))
dx_prev = dx.getVarOffset(-1)
prb.createCost('min_dx_prev', dx_prev, nodes=list(range(N, N+1)))


# Constraints
du_prev = du.getVarOffset(-1)
x_int = F_integrator(x0=dx_prev, p=du_prev, time=dt)
prb.createConstraint("multiple_shooting", x_int["xf"] - dx, nodes=list(range(1, N+1)))

# SQP solver requires cost function in form of residual!
opts = {#SQP
        'max_iter': 100, "solution_convergence": 1e-9,
        #QPOASES
        'sparse': True, 'hessian_type': 'posdef', 'printLevel': 'none', 'linsol_plugin': 'ma57'}

solver = solver.Solver.make_solver('gnsqp', prb, opts)
solver.set_iteration_callback()
solver.solve()

t = time.time()



solver.solve()
solution = solver.getSolutionDict()
print ("first solve: ", time.time() - t)


print ("compute Hessian time: ", solver.getHessianComputationTime())
print ("compute QP time: ", solver.getQPComputationTime())

dx_hist = solution['dx']
du_hist = solution['du']

obj_history = solver.getObjectiveIterations()
con_history = solver.getConstraintNormIterations()

# Retrieve the solution
x0_opt = dx_hist[0, :]
x1_opt = dx_hist[1, :]
u_opt = du_hist.T

# Plot the results
plt.figure(1)
plt.clf()
plt.subplot(121)
plt.plot(np.linspace(0, T, N+1), x0_opt, '--')
plt.plot(np.linspace(0, T, N+1), x1_opt, '-')
plt.step(np.linspace(0, T, N), u_opt, '-.')
plt.title("Solution: Gauss-Newton SQP")
plt.xlabel('time')
plt.legend(['x0 trajectory', 'x1 trajectory', 'u trajectory'])
plt.grid()

plt.subplot(122)
plt.title("SQP solver output")
plt.semilogy(obj_history)
plt.semilogy(con_history)
plt.xlabel('iteration')
plt.legend(['Objective value', 'Constraint violation'], loc='center right')
plt.grid()

plt.show()
