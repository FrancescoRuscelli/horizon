from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import time
import os, math
from itertools import filterfalse
import argparse

# command line arguments
parser = argparse.ArgumentParser('Horizon\'s twist jump motion with BostonDynamics SpotMini robot.')
parser.add_argument('--solver', '-s', default='ilqr', help='Solver type (ilqr, gnsqp, ipopt)')
parser.add_argument('--codegen', '-c', action='store_true', help='Code-generate problem prior to solution (only ilqr for now)')
parser.add_argument('--verbose', '-v', action='store_true', help='Print more output')
parser.add_argument('--interactive', '-i', action='store_true', help='Plot each inner iteration of the solver (only ilqr for now)')
parser.add_argument('--resample', '-r', action='store_true', help='Resample solution prior to replaying it')

args = parser.parse_args()
parser.print_help()

# output mat file 
filename_with_ext = __file__
filename, _ = os.path.splitext(filename_with_ext)
basename = os.path.basename(filename)
ms = mat_storer.matStorer(f'/tmp/{basename}.mat')
print(f'will save solution to /tmp/{basename}.mat')

# options
solver_type = args.solver
transcription_method = 'multiple_shooting'
transcription_opts = dict(integrator='RK4')
tf = 2.0
n_nodes = 100
ilqr_plot_iter = args.interactive
t_jump = (1.0, 1.5)

# load urdf
urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

# parameters
n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3
dt = tf/n_nodes

# define dynamics
prb = problem.Problem(n_nodes)
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)
f_list = [prb.createInputVariable(f'f{i}', n_f) for i in range(n_c)]
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)
prb.setDt(dt)

# contact map
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, f_list))

# initial state and initial guess
q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.52,
                   0.0, 0.9, -1.52,
                   0.0, 0.9, -1.52,
                   0.0, 0.9, -1.52])

q.setBounds(q_init, q_init, 0)
q_dot.setBounds(np.zeros(n_v), np.zeros(n_v), 0)
q.setInitialGuess(q_init)

for f in f_list:
    f.setInitialGuess([0, 0, 55])

# transcription (if not ilqr)
if solver_type != 'ilqr':
    th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# dynamic feasibility
pin_ref_frame = cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED
id_fn = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), pin_ref_frame)
tau = id_fn.call(q, q_dot, q_ddot, contact_map)
prb.createIntermediateConstraint("dynamic_feasibility", tau[:6])

# gauss-newton sqp works with residuals!
# tbd: check this under the hood
if solver_type == 'gnsqp':
    def residual_to_cost(r):
        return r
else:
    def residual_to_cost(r):
        return cs.sumsqr(r)

# final velocity is zero
prb.createFinalConstraint('final_velocity', q_dot)

# final pose
q_tgt = q_init.copy()
q_tgt[0] = 0
q_tgt[5] = math.sin(math.pi/4)  # twist jump 90 deg!
prb.createFinalConstraint('q_fb', q[:6] - q_tgt[:6])
prb.createFinalCost('q_f', residual_to_cost(10*(q[7:] - q_tgt[7:])))

# contact handling
k_all = range(1, n_nodes+1)
k_swing = list(range(*[int(t/dt) for t in t_jump]))
k_stance = list(filterfalse(lambda k: k in k_swing, k_all))
lifted_legs = contacts_name.copy()

# contact velocity is zero, and normal force is positive
for frame, f in contact_map.items():
    nodes = k_stance if frame in lifted_legs else k_all
    
    FK = cs.Function.deserialize(kindyn.fk(frame))
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    
    p = FK(q=q)['ee_pos']
    p_start = FK(q=q_init)['ee_pos']
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']

    # node: on first swing node vel should be zero!
    prb.createConstraint(f"{frame}_vel", v, nodes=list(nodes) + [k_swing[0]])


# swing force is zero
for leg in lifted_legs:
    fzero = np.zeros(n_f)
    contact_map[leg].setBounds(fzero, fzero, nodes=k_swing)

# cost
prb.createCost("min_q", residual_to_cost(1e-1 * (q[7:] - q_init[7:])))
prb.createIntermediateCost("min_q_ddot", residual_to_cost(1e-3* (q_ddot)))
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", residual_to_cost(1e-3 * (f)))


# =============
# SOLVE PROBLEM
# =============

opts = dict()
if solver_type == 'ipopt':
    opts['ipopt.tol'] = 0.001
    opts['ipopt.constr_viol_tol'] = n_nodes * 1e-12
    opts['ipopt.max_iter'] = 2000
    opts['ipopt.linear_solver'] = 'ma57'

if solver_type == 'ilqr':
    opts = {
            'ilqr.max_iter': 1000,
            'ilqr.integrator': 'RK4', 
            'ilqr.svd_threshold': 1e-12,
            'ilqr.kkt_decomp_type': 'ldlt',
            'ilqr.constr_decomp_type': 'qr',
            'ilqr.codegen_enabled': args.codegen,
            'ilqr.codegen_workdir': f'/tmp/{basename}',
            }

if solver_type == 'gnsqp':
    qp_solver = 'osqp'
    if qp_solver == 'osqp':
        opts['gnsqp.qp_solver'] = 'osqp'
        opts['warm_start_primal'] = True
        opts['warm_start_dual'] = True
        opts['merit_derivative_tolerance'] = 1e-6
        opts['constraint_violation_tolerance'] = n_nodes * 1e-12
        opts['osqp.polish'] = True # without this
        opts['osqp.delta'] = 1e-9  # and this, it does not converge!
        opts['osqp.verbose'] = False
        opts['osqp.rho'] = 0.02
        opts['osqp.scaled_termination'] = False
  
t = time.time()
solver = solver.Solver.make_solver(solver_type, prb, opts)
elapsed = time.time() - t
print(f'built problem in {elapsed} s')

try:
    solver.set_iteration_callback()
    solver.plot_iter = ilqr_plot_iter 
except:
    pass

t = time.time()
solver.solve()
elapsed = time.time() - t
print(f'solved in {elapsed} s')

try:
    solver.print_timings()
except:
    pass

solution = solver.getSolutionDict()
solution_constraints_dict = dict()

if isinstance(dt, cs.SX):
    ms.store({**solution, **solution_constraints_dict})
else:
    dt_dict = dict(dt=dt)
    ms.store({**solution, **solution_constraints_dict, **dt_dict})


# resampling
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))
resampling = args.resample
if resampling:

    if isinstance(dt, cs.SX):
        dt_before_res = solution['dt'].flatten()
    else:
        dt_before_res = dt

    dt_res = 0.001
    dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
    q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
        solution["q"], solution["q_dot"], solution["q_ddot"], dt_before_res, dt_res, dae, contact_map,
        kindyn,
        pin_ref_frame)

    repl = replay_trajectory(dt_res, joint_names, q_res, contact_map_res, pin_ref_frame, kindyn)
else:
    # remember to run a robot_state_publisher
    repl = replay_trajectory(dt, joint_names, solution['q'], contact_map, pin_ref_frame, kindyn)

print('view animated solution with roslaunch horizon_examples spot.launch!')
repl.replay(is_floating_base=True)

