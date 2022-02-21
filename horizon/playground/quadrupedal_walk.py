#!/usr/bin/env python3

import casadi as cs
import numpy as np
from horizon import problem
import horizon
from horizon.solvers import Solver
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils.plotter import PlotterHorizon

import matplotlib.pyplot as plt

from typing import List

class Walker:

    class Step:
        def __init__(self, leg, t_start, t_goal, goal):
            self.leg = leg
            self.t_start = t_start
            self.t_goal = t_goal
            self.goal = goal
    
    def __init__(self, 
                 ns: int, 
                 tf: float,  
                 mass: float, 
                 p0: np.array,
                 contacts: List[np.array],
                 t0=0.0) -> None:

        # main parameters
        self.ns = ns  
        self.tf = tf 
        self.dt = tf/ns
        self.transcription = 'multiple_shooting' 
        # self.transcription = 'direct_collocation' 
        self.solver_type = 'ipopt'

        # physical parameters
        self.g = 9.81
        self.gv = np.array([0, 0, self.g])
        self.mass = mass 
        self.nc = 4
        self.fmin = self.mass*self.g/self.nc*0.25
        self.t0 = t0

        # horizon problem
        self.prb = problem.Problem(self.ns)

        # define state
        p = self.prb.createStateVariable('p', 3)
        v = self.prb.createStateVariable('v', 3)
        self.f = [self.prb.createStateVariable('f' + str(i), 3) for i in range(self.nc)]

        # define input
        fdot = [self.prb.createInputVariable('df' + str(i), 3) for i in range(self.nc)]

        # define params
        self.pc = [self.prb.createParameter('pc' + str(i), 3) for i in range(self.nc)]

        # define dynamics
        a = sum(self.f)/mass - self.gv
        xdot = cs.vertcat(v, a, *fdot)
        self.prb.setDynamics(xdot)
        self.prb.setDt(self.dt)

        # set contact position
        for i in range(self.nc):
            self.pc[i].assign(contacts[i])

        # initialize contact state
        self.contact_state = np.full((4, self.ns-1), 1, dtype=int)

        # set constraints
        # (1) forces must produce zero momentum
        momtot = sum([cs.cross(pci - p, fi) for pci, fi in zip(self.pc, self.f)])
        self.prb.createIntermediateConstraint('euler', momtot, nodes=range(1, self.ns))

        # (2) transcription
        Transcriptor.make_method(self.transcription, self.prb, opts={'integrator': 'RK4'})

        # reset steps
        self.steps: List[Walker.Step] = []
        self._add_steps_to_prb()

        # cost
        self.prb.createIntermediateCost('h', cs.sumsqr(p[2] - p0[2]))
        self.prb.createIntermediateCost('regfd', 1e-6*sum(cs.sumsqr(dfi) for dfi in fdot))
        self.prb.createIntermediateCost('regf', 1e-6*sum(cs.sumsqr(fi) for fi in self.f))

        # initial condition
        p.setBounds(lb=p0, ub=p0, nodes=0)
        v0 = np.zeros(3)
        v.setBounds(lb=v0, ub=v0, nodes=0)

        # final constraint
        v.setBounds(lb=np.zeros(3), ub=np.zeros(3), nodes=ns)

        # create solver
        self.solver = Solver.make_solver(self.solver_type, self.prb,
                        opts={'ipopt.linear_solver': 'ma57', 
                            'ipopt.warm_start_init_point': 'yes'
                        })

        if self.solver_type == 'ilqr':
            self.solver.plot_iter = True
            self.solver.set_iteration_callback()

        print('solving initial problem..')
        self.solver.solve()

    
    def add_step(self, leg: int, t_start: float, t_goal: float, goal: np.array):
        s = Walker.Step(leg, t_start, t_goal, goal)
        self.steps.append(s)


    def solve(self, 
              shift_nodes=0):

        # update starting time
        self.t0 += shift_nodes*self.dt

        # initial guess as shifted optimal solution
        self.prb.getState().setInitialGuess(self.solver.x_opt[:,  self.ns-shift_nodes])
        for n in range(shift_nodes, self.ns+1):
            self.prb.getState().setInitialGuess(self.solver.x_opt[:,  n], nodes=n-shift_nodes)

        self.prb.getInput().setInitialGuess(self.solver.u_opt[:,  self.ns-shift_nodes-1])
        for n in range(shift_nodes, self.ns):
            self.prb.getInput().setInitialGuess(self.solver.u_opt[:,  n], nodes=n-shift_nodes)

        # reset contact position and state
        for i in range(4):
            self.pc[i].assign(self.pc[i].getValues(nodes=shift_nodes))
        
        # add steps
        self._add_steps_to_prb()

        # set initial state
        self.prb.setInitialState(self.solver.x_opt[:, shift_nodes])

        # solve
        self.solver.solve()

    
    def _add_steps_to_prb(self):

        for i in range(self.nc):
            self.f[i][2].setBounds(self.fmin, np.inf)

        for s in self.steps:
            self._make_step(s.leg, s.t_start, s.t_goal, s.goal)

    
    def _make_step(self, leg: int, t_start: float, t_goal: float, goal: np.array):
        
        nodes = [int((t - self.t0)/self.tf*self.ns) for t in (t_start, t_goal)]

        # step before horizon
        if nodes[1] < 0:
            return

        # step after horizon
        if nodes[0] > self.ns-1:
            return 

        nodes[0] = max(0, nodes[0])
        nodes[1] = min(nodes[1], self.ns-1)

        self.f[leg][2].setBounds(lb=0, ub=0, nodes=range(*nodes))
        self.pc[leg].assign(goal, nodes=range(nodes[1], self.ns))
        self.contact_state[leg, range(*nodes)] = 0



walker = Walker(ns=80, 
            tf=4.0,
            mass=110,
            p0=[0.10, 0.0, 0.6],
            contacts=[ 
                [0.4, 0.4, 0],
                [0.4, -0.4, 0],
                [-0.4, -0.4, 0],
                [-0.4, 0.4, 0],
            ])

walker.add_step(leg=1, t_start=0.2, t_goal=0.8, goal=np.array([0.5, -0.4, 0.10]))
walker.add_step(leg=0, t_start=1.0, t_goal=1.7, goal=np.array([0.5, 0.4, 0.10]))
walker.add_step(leg=2, t_start=2.5, t_goal=3.2, goal=np.array([-0.3, -0.4, 0.10]))
walker.add_step(leg=3, t_start=4.5, t_goal=5.5, goal=np.array([-0.3, 0.4, 0.10]))

walker.solve()

plt.figure()
plt.plot(walker.solver.x_opt[:3, :].T)
plt.grid()

solution = walker.solver.getSolutionDict()

plt.figure()
for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.plot(solution['f' + str(i)].T)
    plt.grid()


##################
## second round ##
##################

walker.solve(shift_nodes=walker.ns//4)

plt.figure()
plt.plot(walker.solver.x_opt[:3, :].T)
plt.grid()

plt.figure()
for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.plot(solution['f' + str(i)].T)
    plt.grid()
plt.show()