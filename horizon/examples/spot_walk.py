import time
from typing import List

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os, math
from itertools import filterfalse
import rospkg

# mat storer
filename_with_ext = __file__
filename, _ = os.path.splitext(filename_with_ext)
ms = mat_storer.matStorer(f'{filename}.mat')

# options
solver_type = 'ilqr'
transcription_method = 'multiple_shooting'
transcription_opts = dict(integrator='RK4')
load_initial_guess = False
tf = 10.0
n_nodes = 100
ilqr_plot_iter = False

# load urdf
urdffile = rospkg.RosPack().get_path('spot_urdf') + '/urdf/spot.urdf'
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
dt = tf / n_nodes
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

# define dynamics
prb = problem.Problem(n_nodes)
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)
f_list = [prb.createInputVariable(f'force_{i}', n_f) for i in contacts_name]
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)
prb.setDt(dt)

# contact map
contact_map = dict(zip(contacts_name, f_list))

# initial state and initial guess
q_init = np.array([0.0, 0.0, 0.505, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.52,
                   0.0, 0.9, -1.52,
                   0.0, 0.9, -1.52,
                   0.0, 0.9, -1.52])

q.setBounds(q_init, q_init, 0)
q_dot.setBounds(np.zeros(n_v), np.zeros(n_v), 0)

q.setInitialGuess(q_init)

for f in f_list:
    f.setInitialGuess([0, 0, 55])

# transcription
if solver_type != 'ilqr':
    th = Transcriptor.make_method(transcription_method, prb, dt, opts=transcription_opts)

# dynamic feasibility
id_fn = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
tau = id_fn.call(q, q_dot, q_ddot, contact_map)
prb.createIntermediateConstraint("dynamic_feasibility", tau[:6])

if solver_type == 'gnsqp':
    def residual_to_cost(r):
        return r
else:
    def residual_to_cost(r):
        return cs.sumsqr(r)

# base link vreg
vref = prb.createParameter('vref', 3)
v = cs.vertcat(q_dot[0], q_dot[1], q_dot[5])
prb.createCost('vref', 2 * residual_to_cost(v - vref), nodes=range(1, n_nodes + 1))


# barrier function
def barrier(x):
    return cs.if_else(x > 0, 0, x ** 2)


# z trajectory
def z_trj(tau):
    return 64. * tau ** 3 * (1 - tau) ** 3


# save containers that will need node shifting
contact_constr = list()
unilat_constr = list()
clea_constr = list()
contact_y = list()

zdes_params = list()

# contact velocity is zero, and normal force is positive
for i, frame in enumerate(contacts_name):
    FK = cs.Function.deserialize(kindyn.fk(frame))
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))

    p = FK(q=q)['ee_pos']
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']

    # kinematic contact
    contact = prb.createConstraint(f"{frame}_vel", v, nodes=[])

    # unilateral forces
    fcost = barrier(f_list[i][2] - 10.0)  # fz > 10
    unil = prb.createIntermediateCost(f'{frame}_unil', fcost, nodes=[])

    # clearance
    z_des = prb.createParameter(f'{frame}_z_des', 1)
    clea = prb.createConstraint(f"{frame}_clea", p[2] - z_des, nodes=[])

    # go straight
    p0 = FK(q=q_init)['ee_pos']
    cy = prb.createIntermediateCost(f'{frame}_y', 2 * residual_to_cost(p0[1] - p[1]), nodes=[])
    contact_y.append(cy)

    # add to fn container
    contact_constr.append(contact)
    unilat_constr.append(unil)
    clea_constr.append(clea)
    zdes_params.append(z_des)

# cost
# prb.createCostFunction("min_rot", 10 * cs.sumsqr(q[3:6] - q_init[3:6]))
# prb.createCostFunction("min_xy", 100 * cs.sumsqr(q[0:2] - q_init[0:2]))
prb.createCost("min_q", residual_to_cost(0.05 * (q[7:] - q_init[7:])))
# prb.createCostFunction("min_q_dot", 1e-2 * cs.sumsqr(q_dot))
prb.createIntermediateCost("min_q_ddot", residual_to_cost(1e-2 * (q_ddot)))
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", residual_to_cost(1e-3 * (f)))

# =============
# SOLVE PROBLEM
# =============
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 30,
        'ipopt.linear_solver': 'ma57',
        'ilqr.max_iter': 440,
        'ilqr.alpha_min': 0.1,
        'ilqr.huu_reg': 0.0,
        'ilqr.kkt_reg': 0.0,
        'ilqr.integrator': 'RK4',
        'ilqr.closed_loop_forward_pass': True,
        'ilqr.line_search_accept_ratio': 1e-4,
        'ilqr.kkt_decomp_type': 'ldlt',
        'ilqr.constr_decomp_type': 'qr',
        'ilqr.codegen_enabled': True,
        'ilqr.codegen_workdir': '/tmp/ilqr_walk',
        'gnsqp.qp_solver': 'osqp'
        }

opts['warm_start_primal'] = True
opts['warm_start_dual'] = True
opts['osqp.polish'] = False
opts['osqp.verbose'] = False

solver = solver.Solver.make_solver(solver_type, prb, opts)

try:
    solver.set_iteration_callback()
    solver.plot_iter = ilqr_plot_iter
except:
    pass


# helper class representing a step
class Step:
    def __init__(self, leg, k_start, k_goal, goal=[]):
        self.leg = leg
        self.k_start = k_start
        self.k_goal = k_goal
        self.goal = np.array(goal)


# create a gait pattern
steps = list()
n_steps = 20
pattern = [3, 1, 2, 0]
stride_time = 8.0
duty_cycle = 0.80
tinit = 1.0

for i in range(n_steps):
    l = pattern[i % n_c]
    t_start = tinit + i * stride_time / n_c
    t_goal = t_start + stride_time * (1 - duty_cycle)
    s = Step(leg=l, k_start=int(t_start / dt), k_goal=int(t_goal / dt))
    steps.append(s)
    print(l, t_start, t_goal)


def set_gait_pattern(steps: List[Step], k0: float):
    """
    Set the correct nodes to wpg costs and bounds according
    to a specified gait pattern and initial horizon time (absolute)
    """

    # reset bounds
    for f in f_list:
        f.setBounds(lb=np.full(n_f, -np.inf),
                    ub=np.full(n_f, np.inf))

    # reset contact indices for all legs
    contact_nodes = [list(range(1, n_nodes + 1)) for _ in range(n_c)]
    unilat_nodes = [list(range(n_nodes)) for _ in range(n_c)]
    clea_nodes = [list() for _ in range(n_c)]
    contact_k = [list() for _ in range(n_c)]

    for s in steps:
        s: Step = s
        l = s.leg
        k_start = s.k_start - k0
        k_goal = s.k_goal - k0
        swing_nodes = list(range(k_start, k_goal))
        swing_nodes_in_horizon_x = [k for k in swing_nodes if k >= 0 and k <= n_nodes]
        swing_nodes_in_horizon_u = [k for k in swing_nodes if k >= 0 and k < n_nodes]
        n_swing = len(swing_nodes)

        # this step is outside the horizon!
        if n_swing == 0:
            continue

        # update nodes contact constraint
        contact_nodes[l] = [k for k in contact_nodes[l] if k not in swing_nodes]

        # contact instants
        if k_goal <= n_nodes and k_goal > 0:
            contact_k[l].append(k_goal)

        # update nodes for unilateral constraint
        unilat_nodes[l] = [k for k in unilat_nodes[l] if k not in swing_nodes]

        # update zero force constraints
        fzero = np.zeros(n_f)
        f_list[l].setBounds(lb=fzero, ub=fzero, nodes=swing_nodes_in_horizon_u)

        # update z trajectory constraints
        # for all swing nodes + first stance node
        k_trj = swing_nodes_in_horizon_x[:]
        # k_trj = [k for k in k_trj if k <= n_nodes]
        for k in k_trj:
            tau = (k - k_start) / n_swing
            zdes_params[l].assign(z_trj(tau) * 0.10, nodes=k)

        clea_nodes[l].extend(k_trj)

    for i in range(n_c):
        contact_constr[i].setNodes(contact_nodes[i], erasing=True)
        unilat_constr[i].setNodes(unilat_nodes[i], erasing=True)
        clea_constr[i].setNodes(clea_nodes[i], erasing=True)
        contact_y[i].setNodes(contact_k[i], erasing=True)


def set_initial_guess():
    xig = np.roll(solver.x_opt, -1, axis=1)
    xig[:, -1] = solver.x_opt[:, -1]
    prb.getState().setInitialGuess(xig)

    uig = np.roll(solver.u_opt, -1, axis=1)
    uig[:, -1] = solver.u_opt[:, -1]
    prb.getInput().setInitialGuess(uig)

    prb.setInitialState(x0=xig[:, 0])


vref.assign([0.05, 0, 0])
k0 = 0
set_gait_pattern(steps=steps, k0=k0)
t = time.time()
solver.solve()
elapsed = time.time() - t
print(f'solved in {elapsed} s')

solution = solver.getSolutionDict()
contact_map = {contacts_name[i]: solution[f_list[i].getName()] for i in range(n_c)}
repl = replay_trajectory(dt, joint_names, solution['q'], contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED,
                         kindyn)

solver.max_iter = 1
while True:
    vref.assign([0.05, 0, 0])
    k0 += 1
    set_initial_guess()
    set_gait_pattern(steps=steps, k0=k0)
    t = time.time()
    solver.solve()
    elapsed = time.time() - t
    print(f'solved in {elapsed} s')

    solution = solver.getSolutionDict()
    repl.frame_force_mapping = {contacts_name[i]: solution[f_list[i].getName()][:, 0:1] for i in range(n_c)}
    repl.publish_joints(solution['q'][:, 0])
    repl.publishContactForces(rospy.Time.now(), solution['q'][:, 0], 0)

    for f in f_list:
        print(f'{f.getName()} = {solution[f.getName()][2, 0]}')

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

# ========================================================
plot_all = False
plot_fun = False
plot_forces = False

if plot_forces:
    for f in [f_list[i].getName() for i in range(len(contacts_name))]:
        plt.figure()
        for dim in range(solution[f].shape[0]):
            plt.plot(np.array(range(solution[f].shape[1])), solution[f][dim, :])

        plt.title(f'force {f}')

    plt.show()

if plot_fun:
    hplt = plotter.PlotterHorizon(prb, solution)
    # hplt.plotVariables(show_bounds=True, legend=False)
    hplt.plotFunctions(show_bounds=True)
    # hplt.plotFunction('inverse_dynamics', show_bounds=True, legend=True, dim=range(6))
    plt.show()

if plot_all:
    pos_contact_list = list()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']
        plt.figure()
        plt.title(contact)
        plt.plot(pos.toarray().T)

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xy')
        plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xz')
        plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)

    plt.show()
# ======================================================

contact_map = {contacts_name[i]: solution[f_list[i].getName()] for i in range(n_c)}

# resampling
resampling = False
if resampling:

    if isinstance(dt, cs.SX):
        dt_before_res = solution['dt'].flatten()
    else:
        dt_before_res = dt

    dt_res = 0.01
    dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
    q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
        solution["q"], solution["q_dot"], solution["q_ddot"], dt_before_res, dt_res, dae, contact_map,
        kindyn,
        cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

    repl = replay_trajectory(dt_res, joint_names, q_res, contact_map_res, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED,
                             kindyn)
else:
    # remember to run a robot_state_publisher
    repl = replay_trajectory(dt, joint_names, solution['q'], contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED,
                             kindyn)

repl.sleep(1.)
repl.replay(is_floating_base=True)