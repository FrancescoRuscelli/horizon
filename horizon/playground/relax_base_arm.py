
from horizon.problem import Problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.ros import replay_trajectory
from horizon.transcriptions.transcriptor import Transcriptor
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon.solvers import solver
import os, argparse
from itertools import filterfalse
import numpy as np
import casadi as cs

from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot

import rospkg, rospy
import time

def rot_err(R, Rdes):
    Re = cs.mtimes(Rdes.T, R)
    S = (Re - Re.T)/2.0
    err = cs.vertcat(S[2, 1], S[0, 2], S[1, 0])
    return -err

tf = 2.0
N = 50
dt = tf/N
solver_type = 'ilqr'
qinit = {
    'relax_arm1_joint1': 1.0,
    'relax_arm1_joint2': -1.8,
    'relax_arm1_joint4': 0.8,
}

# kin dyn
urdf_path = rospkg.RosPack().get_path('relax_urdf') + '/urdf/relax_xbot2.urdf'
with open(urdf_path, 'r') as f:
    urdf = f.read()

# cartesio 
srdf_path = rospkg.RosPack().get_path('relax_srdf') + '/srdf/relax.srdf'
with open(srdf_path, 'r') as f:
    srdf = f.read()



rospy.set_param('robot_description', urdf)

kd = cas_kin_dyn.CasadiKinDyn(urdf)
kd_frame = cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED

# joint names
joint_names = kd.joint_names()
if 'reference' in joint_names: joint_names.remove('reference')
if 'universe' in joint_names: joint_names.remove('universe')
print(joint_names)

prb = Problem(N=N)
q = prb.createStateVariable('q', kd.nq())
dq = prb.createStateVariable('dq', kd.nv())
u = prb.createInputVariable('ddq', kd.nv() - 4)  # no y, z, rotx, roty ( = 6 + 2 = 8 input size )
ddq = cs.vertcat(u[0], 0, 0, 0, 0, u[1], u[2:])

# dynamics
x, x_dot = utils.double_integrator_with_floating_base(q, dq, ddq)
prb.setDynamics(x_dot)
prb.setDt(dt)

# useful vars
vzero = np.zeros(kd.nv())

# initial state
qinit = kd.mapToQ(qinit)
qmax = np.array([2.6, 2.6, 2.6, 2.6, 2.2, 2.6])
q[7:].setBounds(-qmax, qmax)
q.setBounds(qinit, qinit, nodes=0)
dq.setBounds(vzero, vzero, nodes=0)

# guess
q.setInitialGuess(qinit)

# feasibility
# prb.createIntermediateConstraint('feas', ddq[1:5])

# transcription
if solver_type != 'ilqr':
    Transcriptor.make_method('multiple_shooting', prb)

# regularization
prb.createIntermediateResidual('regq', 1e-2*(q[7:] - qinit[7:]))
prb.createIntermediateResidual('regv', 3e-1*dq)
prb.createIntermediateResidual('rega', 4e-2*ddq)
prb.createIntermediateResidual('rega_base', 5e-1*ddq[:6])


# target
base_xy_tgt = np.array([0.0, 0.1])
# prb.createFinalConstraint('base_tgt', q[0:2] - base_xy_tgt)
# q[5].setInitialGuess(0.7)

# ee tracking
ee_fk = cs.Function.deserialize(kd.fk('relax_arm1_linkEndEffector'))
ee_pos = ee_fk(q=q)['ee_pos']
ee_rot = ee_fk(q=q)['ee_rot']

ee_pos_tgt = prb.createParameter('ee_pos_tgt', 3)
ee_pos_tgt.assign(ee_fk(q=qinit)['ee_pos'])

ee_rot_tgt = prb.createParameter('ee_rot_tgt', 9)
ee_rot_tgt.assign(ee_fk(q=qinit)['ee_rot'].reshape((9, 1)))

prb.createResidual('ee_pos', ee_pos - ee_pos_tgt)
prb.createResidual('ee_rot', 0.5*rot_err(ee_rot, ee_rot_tgt.reshape((3,3))))
# prb.createFinalConstraint('ee_pos', ee_pos - ee_tgt)


# final vel
prb.createFinalConstraint('final_v_base', dq[[0, 5]])
prb.createFinalConstraint('final_v_arm', dq[-6:])

# solver
solv = solver.Solver.make_solver(solver_type, prb, 
    {'ilqr.alpha_min': 0.01, 
    'ilqr.use_filter': False, 
    'ilqr.enable_line_search': False,
    'ilqr.step_length_threshold': 1e-9,
    'ilqr.line_search_accept_ratio': 0.0,
    'ilqr.max_iter': 200,
    'ilqr.rho_base': 1e1,
    'ilqr.verbose': True,
    'ilqr.enable_gn': True,
    'ilqr.codegen_enabled': True, 
    'ilqr.codegen_workdir': '/tmp/relax_codegen'})

try:
    solv.set_iteration_callback()
except:
    pass




class CartesioSolver:
    
    def __init__(self, urdf, srdf, pb) -> None:

        kd_frame = cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED
        self.base_fk = cs.Function.deserialize(kd.fk('base_link'))
        self.base_dfk = cs.Function.deserialize(kd.frameVelocity('base_link', kd_frame))
        
        self.model = xbot.ModelInterface(get_xbot_config(urdf=urdf, srdf=srdf))

        self.set_model_from_solution()
        
        self.ci = pyci.CartesianInterface.MakeInstance(
            solver='',
            problem=pb,
            model=self.model, 
            dt=0.01)
        
        roscpp.init('ciao', [])
        self.ciros = pyci.RosServerClass(self.ci)
        
        self.task = self.ci.getTask('relax_arm1_linkEndEffector')

    def set_model_from_solution(self):
        q = solv.getSolutionDict()['q'][:, 1]
        dq = solv.getSolutionDict()['dq'][:, 1]
        base_pos = self.base_fk(q=q)['ee_pos'].toarray()
        base_rot = self.base_fk(q=q)['ee_rot'].toarray()

        qmdl = np.zeros(self.model.getJointNum())        
        qmdl[6:] = q[7:]
        base_pose = Affine3(pos=base_pos)
        base_pose.linear = base_rot 
        self.model.setJointPosition(qmdl)
        self.model.setFloatingBasePose(base_pose)
        self.model.update()

    def solve(self):

        Tref = self.task.getPoseReference()[0]
        ee_pos_tgt.assign(Tref.translation)
        ee_rot_tgt.assign(Tref.linear.T.reshape((9, 1)))

        tic = time.time()
        solv.solve()
        toc = time.time() 
        print(f'solved in {toc-tic}')

        xig = np.roll(solv.x_opt, -1, axis=1)
        xig[:, -1] = solv.x_opt[:, -1]
        prb.getState().setInitialGuess(xig)

        uig = np.roll(solv.u_opt, -1, axis=1)
        uig[:, -1] = solv.u_opt[:, -1]
        prb.getInput().setInitialGuess(uig)

        prb.setInitialState(x0=xig[:, 0])

        self.set_model_from_solution()
        self.ciros.run()





pb = {
    'stack': [
        ['ee']
    ],
    'ee': {
        'type': 'Cartesian',
        'distal_link': 'relax_arm1_linkEndEffector',
    }

}

import yaml
pb = yaml.dump(pb)
print(pb)

solv.solve()    

ci = CartesioSolver(urdf, srdf, pb)

# prb_dict = prb.save()
# prb_dict['dynamics'] = solv.dyn.serialize()


# solution = solv.getSolutionDict()

# from matplotlib import pyplot as plt

# ee_err = ee_fk(q=solution['q'])['ee_pos'].toarray() - np.atleast_2d(ee_tgt.getValues(nodes=0))

# plt.figure()
# plt.plot(solution['q'][[0, 1, 5], :].T)

# plt.figure()
# plt.plot(solution['dq'][[0, 1, 5], :].T)

# plt.figure()
# plt.plot(solution['ddq'][[0, 1, 5], :].T)

# plt.figure()
# plt.plot(solution['q'][7:, :].T)

# plt.figure()
# plt.plot(ee_err.T)

# plt.show()

solv.max_iter = 1

while True:
    ci.solve()
    rospy.sleep(0.001)