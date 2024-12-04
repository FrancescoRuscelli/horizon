import threading
from casadi_kin_dyn import pycasadi_kin_dyn
import rospkg
import casadi as cs
import numpy as np
from typing import List, Dict
import time
import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os
from horizon.problem import Problem
from horizon.solvers import Solver
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils import utils, kin_dyn, mat_storer, resampler_trajectory
from horizon.ros import replay_trajectory

# helper class representing a step
class Step:
    def __init__(self, leg: int, k_start: int, k_goal: int,
                 start=np.array([]), goal=np.array([]),
                 clearance=0.10):
        self.leg = leg
        self.k_start = k_start
        self.k_goal = k_goal
        self.goal = np.array(goal)
        self.start = np.array(start)
        self.clearance = clearance

# main wpg class
class HorizonWpg:

    def __init__(self,
                 urdf,
                 contacts: List[str],
                 fixed_joints: List[str],
                 q_init: Dict[str, float],
                 base_init: np.array,
                 opt: Dict[str, any]) -> None:

        ## robot description

        self.urdf = urdf.replace('continuous', 'revolute')
        self.kd = pycasadi_kin_dyn.CasadiKinDyn(self.urdf)
        self.kd_frame = pycasadi_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED

        self.joint_names = self.kd.joint_names()[2:]

        q_init = {k: v for k, v in q_init.items() if k not in fixed_joints}

        self.contacts = contacts.copy()

        self.nq = self.kd.nq()
        self.nv = self.kd.nv()
        self.nc = len(self.contacts)
        self.nf = 3

        # initial guess (also initial condition and nominal pose)
        q0 = self.kd.mapToQ(q_init)
        q0[:7] = base_init
        v0 = np.zeros(self.nv)

        ## gait params
        f0 = opt.get('f0', np.array([0, 0, 55]))
        self.fmin = opt.get('fmin', 0)
        self.steps = list()

        ## problem description

        # horizon length and time duration
        self.N = opt.get('N', 50)
        self.tf = opt.get('tf', 10.0)
        self.dt = self.tf / self.N

        self.prb = Problem(self.N, receding=True)
        self.prb.setDt(self.dt)


        # state and control vars
        self.q = self.prb.createStateVariable('q', self.nq)
        self.v = self.prb.createStateVariable('v', self.nv)
        self.a = self.prb.createInputVariable('a', self.nv)
        self.forces = [self.prb.createInputVariable('f_' + c, self.nf) for c in contacts]
        self.fmap = {k: v for k, v in zip(self.contacts, self.forces)}

        # dynamics ODE
        _, self.xdot = utils.double_integrator_with_floating_base(self.q, self.v, self.a)
        self.prb.setDynamics(self.xdot)

        # underactuation constraint
        id_fn = kin_dyn.InverseDynamics(self.kd, contacts, self.kd_frame)
        tau = id_fn.call(self.q, self.v, self.a, self.fmap)
        self.prb.createIntermediateConstraint('dynamics', tau[:6])

        # final goal (a.k.a. integral velocity control)
        self.ptgt_final = [0, 0, 0]
        self.vmax = [0.05, 0.05, 0.05]
        self.ptgt = self.prb.createParameter('ptgt', 3)

        goalx = self.prb.createFinalConstraint("final_x", self.q[0] - self.ptgt[0])
        goaly = self.prb.createFinalResidual("final_y", 1e3 * (self.q[1] - self.ptgt[1]))
        goalrz = self.prb.createFinalResidual("final_rz", 1e3 * (self.q[5] - self.ptgt[2]))
        self.base_goal_tasks = [goalx, goaly, goalrz]

        # final velocity
        self.v.setBounds(v0, v0, nodes=self.N)

        # regularization costs

        # base rotation
        self.prb.createResidual("min_rot", 1e-3 * (self.q[3:5] - q0[3:5]))

        # joint posture
        self.prb.createResidual("min_q", 1e0 * (self.q[7:] - q0[7:]))

        # joint velocity
        self.prb.createResidual("min_v", 1e-2 * self.v)

        # final posture
        self.prb.createFinalResidual("min_qf", 1e0 * (self.q[7:] - q0[7:]))

        # regularize input
        self.prb.createIntermediateResidual("min_q_ddot", 1e-1 * self.a)

        for f in self.forces:
            self.prb.createIntermediateResidual(f"min_{f.getName()}", 1e-3 * (f - f0))

        # costs and constraints implementing a gait schedule
        self.contact_constr = list()
        self.unilat_constr = list()
        self.friction_constr = list()
        self.foot_z_constr = list()
        self.foot_tgt_params = list()
        self.foot_tgt_constr = list()
        self.fk_fn = list()
        self.com_fn = cs.Function.deserialize(self.kd.centerOfMass())

        # save default foot height
        self.default_foot_z = list()

        # contact velocity is zero, and normal force is positive
        for i, frame in enumerate(self.contacts):
            # fk functions and evaluated vars
            fk = cs.Function.deserialize(self.kd.fk(frame))
            dfk = cs.Function.deserialize(self.kd.frameVelocity(frame, self.kd_frame))
            self.fk_fn.append(fk)

            ee_p = fk(q=self.q)['ee_pos']
            ee_rot = fk(q=self.q)['ee_rot']
            ee_v = dfk(q=self.q, qdot=self.v)['ee_vel_linear']

            # save foot height
            self.default_foot_z.append(fk(q=q0)['ee_pos'][2])

            # vertical contact frame
            rot_err = cs.sumsqr(ee_rot[2, :2])
            self.prb.createIntermediateCost(f'{frame}_rot', 1e-1 * rot_err)

            # kinematic contact
            contact = self.prb.createConstraint(f"{frame}_vel", ee_v, nodes=[])

            # unilateral forces
            fcost = self._barrier(self.forces[i][2] - self.fmin)  # fz > 10
            unil = self.prb.createIntermediateCost(f'{frame}_unil', 1e-3 * fcost, nodes=[])

            # friction
            mu = opt.get('friction_coeff', 0.5)
            fcost = self._barrier(self.forces[i][2] ** 2 * mu ** 2 - cs.sumsqr(self.forces[i][:2]))
            fc = self.prb.createIntermediateCost(f'{frame}_fc', 1e-3 * fcost, nodes=[])

            # clearance
            foot_tgt = self.prb.createParameter(f'{frame}_foot_tgt', 3)
            clea = self.prb.createConstraint(f"{frame}_z_trj", ee_p[2] - foot_tgt[2], nodes=[])

            # xy goal
            foot_goal = self.prb.createConstraint(f"{frame}_xy_tgt", ee_p[:2] - foot_tgt[:2], nodes=[])


            # add to fn container
            self.contact_constr.append(contact)
            self.unilat_constr.append(unil)
            self.friction_constr.append(fc)
            self.foot_z_constr.append(clea)
            self.foot_tgt_params.append(foot_tgt)
            self.foot_tgt_constr.append(foot_goal)

        ## transcription method
        solver_type = opt.get('solver_type', 'ilqr')

        if solver_type != 'ilqr':
            Transcriptor.make_method('multiple_shooting', self.prb)

        ## solver

        opts = {'ipopt.tol': 0.001,
                'ipopt.constr_viol_tol': 0.001,
                'ipopt.max_iter': 2000}  #

        # set initial condition and initial guess
        self.q.setBounds(q0, q0, nodes=0)
        self.v.setBounds(v0, v0, nodes=0)

        self.q.setInitialGuess(q0)

        (f.setInitialGuess(f0) for f in self.forces)

        # set initial gait pattern
        self._set_gait_pattern(k0=0)

        # create solver and solve initial seed
        self.solver_bs = Solver.make_solver(solver_type, self.prb, opts)
        self.solver_rti = Solver.make_solver('ipopt', self.prb, opts)

        self.bootstrap()

        solution = dict()
        solution['q'] = np.array([])
        contact_map = {k: None for k in self.contacts}

        # visualize bootstrap solution
        # import subprocess
        # solution = self.solver_bs.getSolutionDict()
        # contact_map = dict(zip(contacts, [solution['f_' + c] for c in contacts]))
        # os.environ['ROS_PACKAGE_PATH'] += ':' + path_to_examples
        # subprocess.Popen(["roslaunch", path_to_examples + "/replay/launch/launcher.launch", 'robot:=spot'])
        # rospy.loginfo("'spot' visualization started.")

        self.repl = replay_trajectory.replay_trajectory(0.01,
                                                        self.joint_names, solution['q'],
                                                        contact_map, self.kd_frame, self.kd)

        # self.repl.replay(is_floating_base=True)

        # exit()

    def bootstrap(self):
        t = time.time()
        self.solver_bs.solve()
        elapsed = time.time() - t
        print(f'bootstrap solved in {elapsed} s')
        self.solution = self.solver_bs.getSolutionDict()
        self.update_initial_guess(dk=0)

    def rti(self, k):

        self._set_gait_pattern(k0=k)

        t = time.time()
        self.solver_rti.solve()
        elapsed = time.time() - t
        print(f'rti solved in {elapsed} s')

        self.solution = self.solver_rti.getSolutionDict()

    def publish_solution(self):

        self.repl.frame_force_mapping = {self.contacts[i]: self.solution[self.forces[i].getName()] for i in
                                         range(self.nc)}
        self.repl.publish_joints(self.solution['q'][:, 0])
        self.repl.publishContactForces(rospy.Time.now(), self.solution['q'][:, 0], 0)

    def update_initial_guess(self, dk=1):
        x_opt = self.solution['x_opt']
        u_opt = self.solution['u_opt']
        xig = np.roll(x_opt, -dk, axis=1)
        for i in range(abs(dk)):
            xig[:, -1 - i] = x_opt[:, -1]
        self.prb.getState().setInitialGuess(xig)

        uig = np.roll(u_opt, -dk, axis=1)
        for i in range(abs(dk)):
            uig[:, -1 - i] = u_opt[:, -1]
        self.prb.getInput().setInitialGuess(uig)

        self.prb.setInitialState(x0=xig[:, 0])

    def resample(self, dt_res):

        contact_map = {self.contacts[i]: self.solution[self.forces[i].getName()] for i in range(self.nc)}

        dae = {'x': self.prb.getState().getVars(), 'p': self.a, 'ode': self.xdot, 'quad': 0.0}

        q_res, v_res, a_res, f_res, tau_res = resampler_trajectory.resample_torques(
            self.solution['q'], self.solution['v'], self.solution['a'],
            self.dt, dt_res,
            dae,
            contact_map,
            self.kd, self.kd_frame
        )

        self.solution['q_res'] = q_res
        self.solution['v_res'] = v_res
        self.solution['a_res'] = a_res
        self.solution['tau_res'] = tau_res
        self.solution['f_res'] = f_res

        return q_res, v_res, a_res, tau_res, f_res

    def replay(self, dt_res=0.01):

        q_res, v_res, a_res, tau_res, f_res = self.resample(dt_res)
        repl = replay_trajectory.replay_trajectory(dt_res, self.joint_names, q_res, f_res, self.kd_frame, self.kd)
        repl.replay()

    # barrier function
    def _barrier(self, x):
        return cs.sum1(cs.if_else(x > 0, 0, x ** 2))

    # z trajectory
    def _z_trj(self, tau):
        return 64. * tau ** 3 * (1 - tau) ** 3

    def _set_step_ctrl(self):

        for t in self.base_goal_tasks:
            t.setNodes([], erasing=True)

    def _set_base_ctrl(self):

        for t in self.base_goal_tasks:
            t.setNodes([self.N], erasing=True)

    def _set_gait_pattern(self, k0: int):
        """
        Set the correct nodes to wpg costs and bounds according
        to a specified gait pattern and initial horizon time (absolute)
        """

        # reset bounds
        for f in self.forces:
            f.setBounds(lb=np.full(self.nf, -np.inf),
                        ub=np.full(self.nf, np.inf))

        # reset contact indices for all legs
        contact_nodes = [list(range(1, self.N + 1)) for _ in range(self.nc)]
        unilat_nodes = [list(range(self.N)) for _ in range(self.nc)]
        clea_nodes = [list() for _ in range(self.nc)]
        contact_k = [list() for _ in range(self.nc)]

        for s in self.steps:
            s: Step = s
            l = s.leg
            k_start = s.k_start - k0
            k_goal = s.k_goal - k0
            swing_nodes = list(range(k_start, k_goal))
            swing_nodes_in_horizon_x = [k for k in swing_nodes if k >= 0 and k <= self.N]
            swing_nodes_in_horizon_u = [k for k in swing_nodes if k >= 0 and k < self.N]
            n_swing = len(swing_nodes)

            # this step is outside the horizon!
            if n_swing == 0:
                continue

            # update nodes contact constraint
            contact_nodes[l] = [k for k in contact_nodes[l] if k not in swing_nodes and k < self.N]

            # update nodes for unilateral constraint
            unilat_nodes[l] = [k for k in unilat_nodes[l] if k not in swing_nodes]

            # update zero force constraints
            fzero = np.zeros(self.nf)
            self.forces[l].setBounds(lb=fzero, ub=fzero, nodes=swing_nodes_in_horizon_u)

            # update z trajectory constraints
            # for all swing nodes + first stance node
            k_trj = swing_nodes_in_horizon_x[:]

            # compute swing trj
            for k in k_trj:
                tau = (k - k_start) / n_swing
                z_start = self.default_foot_z[l] if s.start.size == 0 else s.start[2]
                z_goal = self.default_foot_z[l] if s.goal.size == 0 else s.goal[2]
                zk = self._z_trj(tau) * s.clearance
                zk += (1 - tau) * z_start + tau * z_goal
                self.foot_tgt_params[l].assign([0, 0, zk], nodes=k)

            if len(k_trj) > 0:
                clea_nodes[l].extend(k_trj)

            # assign xy goal
            if k_goal <= self.N and k_goal > 0 and s.goal.size > 0:
                contact_k[l].append(k_goal)
                self.foot_tgt_params[l].assign(s.goal, nodes=k_goal)

        for i in range(self.nc):
            # print(f'setting contact nodes to: {contact_nodes[i]}')
            self.contact_constr[i].setNodes(contact_nodes[i], erasing=True)
            # print(f'setting unilat nodes to: {unilat_nodes[i]}')
            self.unilat_constr[i].setNodes(unilat_nodes[i], erasing=True)  # fz > 10
            # friction_constr[i].setNodes(unilat_nodes[i], erasing=True)
            # print(f'setting foot_z nodes to: {clea_nodes[i]}')
            self.foot_z_constr[i].setNodes(clea_nodes[i], erasing=True)
            # print(f'setting foot_tgt nodes to: {contact_k[i]}')
            self.foot_tgt_constr[i].setNodes(contact_k[i], erasing=True)


if __name__ == '__main__':


    path_to_examples = os.path.abspath(__file__ + "/../../../examples/")
    os.environ['ROS_PACKAGE_PATH'] += ':' + path_to_examples

    urdffile = os.path.join(path_to_examples, 'urdf', 'spot.urdf')
    urdf = open(urdffile, 'r').read()

    # contact frames
    contacts = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

    # initial state and initial guess
    q_init = {
        'lf_haa_joint': 0.0,
        'lf_hfe_joint': 0.9,
        'lf_kfe_joint': -1.52,
        'rf_haa_joint': 0.0,
        'rf_hfe_joint': 0.9,
        'rf_kfe_joint': -1.52,
        'lh_haa_joint': 0.0,
        'lh_hfe_joint': 0.9,
        'lh_kfe_joint': -1.52,
        'rh_haa_joint': 0.0,
        'rh_hfe_joint': 0.9,
        'rh_kfe_joint': -1.52,
    }

    base_init = np.array([0.0, 0.0, 0.505, 0.0, 0.0, 0.0, 1.0])

    wpg = HorizonWpg(urdf=urdf,
                     contacts=contacts,
                     fixed_joints=[],
                     opt={
                         'fmin': 55.0,
                         'tf': 7.0,
                     },
                     q_init=q_init,
                     base_init=base_init
                     )

    # current iteration
    k = 0

    # l = 0 #int(input('I am sorry to bother you, which leg should I move ?'))
    # cl = 0.1 #float(input('I am sorry to bother you, which clearance should I set ?'))

    # # create a gait pattern
    # s = Step(leg=l, k_start=k + int(2.0 / wpg.dt), k_goal=k + int(4.0 / wpg.dt))
    # s.clearance = cl
    #
    # # set it to wpg
    # wpg.steps = [s]
    # wpg.bootstrap()
    #
    # # visualize bootstrap solution
    # import subprocess
    # contact_map = dict(zip(contacts, [wpg.solution['f_' + c] for c in contacts]))
    # os.environ['ROS_PACKAGE_PATH'] += ':' + path_to_examples
    # subprocess.Popen(["roslaunch", path_to_examples + "/replay/launch/launcher.launch", 'robot:=spot'])
    # rospy.loginfo("'spot' visualization started.")
    #
    # repl = replay_trajectory.replay_trajectory(0.01,
    #                                             wpg.joint_names, wpg.solution['q'],
    #                                             contact_map, wpg.kd_frame, wpg.kd)
    #
    # repl.replay(is_floating_base=True)
    #
    # exit()
    # create a gait pattern
    steps = list()
    n_steps = 8
    pattern = [0, 3, 1, 2]
    stride_time = 6.0
    duty_cycle = 0.80
    tinit = 1.0

    for i in range(n_steps):
        l = pattern[i % wpg.nc]
        t_start = tinit + i * stride_time / wpg.nc
        t_goal = t_start + stride_time * (1 - duty_cycle)
        s = Step(leg=l, k_start=k + int(t_start / wpg.dt), k_goal=k + int(t_goal / wpg.dt))
        steps.append(s)

    wpg.steps = steps
    wpg.bootstrap()



# start rti
rate = rospy.Rate(1. / wpg.dt)
while not rospy.is_shutdown():


    wpg.update_initial_guess(dk=1)
    wpg.rti(k=k)
    k += 1

    wpg.publish_solution()

    rate.sleep()