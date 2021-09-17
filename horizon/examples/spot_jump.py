from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os

transcription_method = 'multiple_shooting' # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# OPTIMIZATION PARAMETERS
n_nodes = 50 # number of shooting nodes
dt = 0.01 # time
n_c = 4 # number of contacts
n_q = kindyn.nq()  # dofs NB: 7 DoFs floating base (quaternions)
n_v = kindyn.nv()  # dofs velocity
n_f = 3  # dofs force

# DoF = nq - 7  # Contacts + anchor_rope + rope

# Create horizon problem
prb = problem.Problem(n_nodes)

#  STATE variables
q = prb.createStateVariable("q", n_q)
q_dot = prb.createStateVariable("q_dot", n_v)

# Creates problem CONTROL variables
q_ddot = prb.createInputVariable("q_ddot", n_v) # joint acc as inputs

f1 = prb.createInputVariable("f1", n_f) # foot 1 force input
f2 = prb.createInputVariable("f2", n_f) # foot 2 force input
f3 = prb.createInputVariable("f3", n_f) # foot 3 force input
f4 = prb.createInputVariable("f4", n_f) # foot 4 force input
dt = prb.createInputVariable("dt", 1) # variable dt as input

# Computing dynamics
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)



# jump_heigth = 0.2


q_min = [-10., -10., -10., -1., -1., -1., -1.]
q_min.extend(kindyn.q_min()[7:])
q_min = np.array(q_min)

q_max = [10., 10., 10., 1., 1., 1., 1.]
q_max.extend(kindyn.q_max()[7:])
q_max = np.array(q_max)

q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
          0.0, 0.789798138441726, -1.5238505,
          0.0, 0.789798138441726, -1.5202315,
          0.0, 0.789798138441726, -1.5300265,
          0.0, 0.789798138441726, -1.5253125])

# set bounds and intial guess of q
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, 0)
q.setInitialGuess(q_init)

# set bounds of q_dot
q_dot_lim = 100.*np.ones(n_v)
q_dot_init = np.zeros(n_v)
q_dot.setBounds(-q_dot_lim, q_dot_lim)
q_dot.setBounds(q_dot_init, q_dot_init, 0)

# set bounds of q_ddot
q_ddot_lim = 100.*np.ones(n_v)
q_ddot.setBounds(-q_ddot_lim, q_ddot_lim)

# set bounds of f
f_lim = 10000.*np.ones(n_f)
f1.setBounds(-f_lim, f_lim)
f2.setBounds(-f_lim, f_lim)
f3.setBounds(-f_lim, f_lim)
f4.setBounds(-f_lim, f_lim)

# set bounds of time
dt_min = [0.03] #[s]
dt_max = [0.15] #[s]
dt_init = [dt_min]
dt.setBounds(dt_min, dt_max)
dt.setInitialGuess(dt_init)

lift_node = 10
touch_down_node = 30
jump_heigth = 0.5

q_fb_traj = np.array([q_init[0], q_init[1], q_init[2] + jump_heigth, 0.0, 0.0, 0.0, 1.0])

prb.createCostFunction("jump", 10.*cs.sumsqr(q[0:3] - q_fb_traj[0:3]), nodes=list(range(lift_node, touch_down_node)))
prb.createCostFunction("min_q_dot", 10.*cs.sumsqr(q_dot))

th = Transcriptor.make_method(transcription_method, prb, 1, opts=transcription_opts)



# tau_min = [0., 0., 0., 0., 0., 0.,  # Floating base
#             -1000., -1000., -1000.,  # Contact 1
#             -1000., -1000., -1000.,  # Contact 2
#             -1000., -1000., -1000.,  # Contact 3
#             -1000., -1000., -1000.]  # Contact 4
#
# tau_max = [0., 0., 0., 0., 0., 0.,  # Floating base
#             1000., 1000., 1000.,  # Contact 1
#             1000., 1000., 1000.,  # Contact 2
#             1000., 1000., 1000.,  # Contact 3
#             1000., 1000., 1000.]  # Contact 4
# dd = {'Contact1': f1, 'Contact2': f2, 'Contact3': f3, 'Contact4': f4}
# tau = kin_dyn.InverseDynamics(kindyn, dd.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, qdot, qddot, dd)
# prb.createConstraint("inverse_dynamics", tau, nodes=list(range(0, ns)), bounds=dict(lb=tau_min, ub=tau_max))
# prb.createFinalConstraint('final_velocity', qdot)
exit()

