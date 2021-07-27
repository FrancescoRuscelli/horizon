#!/usr/bin/env python

import logging
import casadi as cs
from horizon import problem
from horizon.utils import integrators
from horizon.solvers import sqp
import matplotlib.pyplot as plt
import numpy as np
import time


N = 21  # Control discretization
T = 10.0  # End time

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
print(f'xdot: {xdot}')
dae = {'x': dx, 'p': du, 'ode': xdot, 'quad': []}
opts = {'tf': T/N}
F_integrator = integrators.RK4(dae, opts, cs.SX)
print(f"F_integrator: {F_integrator}")

# Cost function
prb.createCostFunction('min_v', cs.vertcat(dx, du), nodes=list(range(0, N)))
dx_prev = dx.getVarOffset(-1)
prb.createCostFunction('min_dx_prev', dx_prev, nodes=list(range(N, N+1)))


# Constraints
du_prev = du.getVarOffset(-1)
x_int = F_integrator(x0=dx_prev, p=du_prev)
prb.createConstraint("multiple_shooting", x_int["xf"] - dx, nodes=list(range(1, N+1)))

# SQP solver requires cost function in form of residual!
prb.createProblem()
problem_dict = prb.getProblem()
problem_dict['f'] = prb.function_container.getCostFList()


d = {'verbose': False}
opts = {'max_iter': 10,
        'osqp.osqp': d}

t = time.time()
solver = sqp.sqp('solver', 'qpoases', problem_dict, opts)
prb.setSolver(solver)

solution = prb.solveProblem()
print ("first solve: ", time.time() - t)

print ("compute Hessian time: ", solver.get_hessian_computation_time())
print ("compute QP time: ", solver.get_qp_computation_time())

dx_hist = solution['dx']
du_hist = solution['du']

obj_history = prb.sol['f']
con_history = prb.sol['g']

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


prb.removeCostFunction('min_v')
prb.createCostFunction('min_v', cs.vertcat(10.*dx, du), nodes=list(range(0, N)))
prb.removeCostFunction('min_dx_prev')
prb.createCostFunction('min_dx_prev', 10.*dx_prev, nodes=list(range(N, N+1)))

prb.createProblem()

prb.solver.f(prb.function_container.getCostFList())

solution = prb.solveProblem()
print ("first solve: ", time.time() - t)

print ("compute Hessian time: ", solver.get_hessian_computation_time())
print ("compute QP time: ", solver.get_qp_computation_time())

dx_hist = solution['dx']
du_hist = solution['du']

obj_history = prb.sol['f']
con_history = prb.sol['g']

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