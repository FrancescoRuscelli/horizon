import casadi as cs
from horizon.utils import utils, kin_dyn, mat_storer
import numpy as np
import time
import os
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn

def RK4(dae, opts=None, casadi_type=cs.SX):

  if opts is None:
    opts = dict()
  x = dae['x']
  qddot = dae['p']
  xdot = dae['ode']
  L = dae['quad']

  f_RK = cs.Function('f_RK', [x, qddot], [xdot, L])

  nx = x.size1()
  nv = qddot.size1()

  X0_RK = casadi_type.sym('X0_RK', nx)
  U_RK = casadi_type.sym('U_RK', nv)

  if 'tf' in opts:
    DT_RK = opts['tf']
  else:
    DT_RK = casadi_type.sym('DT_RK', 1)

  X_RK = X0_RK
  Q_RK = 0

  k1, k1_q = f_RK(X_RK, U_RK)
  k2, k2_q = f_RK(X_RK + DT_RK / 2. * k1, U_RK)
  k3, k3_q = f_RK(X_RK + DT_RK / 2. * k2, U_RK)
  k4, k4_q = f_RK(X_RK + DT_RK * k3, U_RK)

  X_RK = X_RK + DT_RK / 6. * (k1 + 2. * k2 + 2. * k3 + k4)
  Q_RK = Q_RK + DT_RK / 6. * (k1_q + 2. * k2_q + 2. * k3_q + k4_q)

  if 'tf' in opts:
    f = cs.Function('F_RK', [X0_RK, U_RK], [X_RK, Q_RK], ['x0', 'p'], ['xf', 'qf'])
  else:
    f = cs.Function('F_RK', [X0_RK, U_RK, DT_RK], [X_RK, Q_RK], ['x0', 'p', 'time'], ['xf', 'qf'])
  return f

# 0 sx
# 1 sx with map
# 2 full mx with map

type_sol = 3

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

N = 50
 # =========================================================
 # =========================================================
 # =========================================================
if type_sol == 0:

  q = cs.SX.sym('q', n_q)
  q_dot = cs.SX.sym('q_dot', n_v)

  u = cs.SX.sym("actuated_torques", n_v - 6)
  tau = cs.vertcat(cs.SX.zeros(6, 1), u)


  # forces
  f_list = list()
  for i in range(n_c):
      f_list.append(cs.SX.sym(f'f{i}', n_f))

  contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
  contact_map = dict(zip(contacts_name, f_list))
  fd = kin_dyn.ForwardDynamics(kindyn, contacts_name, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

  q_ddot = fd.call(q, q_dot, tau, contact_map)

  x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)

  fs = cs.vertcat(*f_list)
  state = cs.vertcat(q, q_dot)
  input = cs.vertcat(u, fs)

  dae = {'x': state, 'p': input, 'ode': x_dot, 'quad': 1}
  integrator = RK4(dae, opts=dict(tf=0.01))

  g_list = list()
  w_list = list()
  for i in range(N):
    q_i = cs.SX.sym(f"q_{i}", n_q)
    q_dot_i = cs.SX.sym(f"q_dot_{i}", n_v)

    u_i = cs.SX.sym(f"u_{i}", n_v - 6)
    tau_i = cs.vertcat(cs.SX.zeros(6, 1), u_i)

    q_ddot_i = fd.call(q_i, q_dot_i, tau_i, contact_map)

    f_list_i = list()
    for k in range(n_c):
      f_list_i.append(cs.SX.sym(f'f{k}_{i}', n_f))


    fs_i = cs.vertcat(*f_list_i)
    input_i = cs.vertcat(u_i, fs_i)

    w_list.append(q_i)
    w_list.append(q_dot_i)
    w_list.append(u_i)
    w_list.append(fs_i)


    x_i, x_dot = utils.double_integrator_with_floating_base(q_i, q_dot_i, q_ddot_i)

    x_int = integrator(x0=x_i, p=input_i)

    g_list.append(x_int["xf"] - x_i)


  w = cs.vertcat(*w_list)
  g = cs.vertcat(*g_list)


  prob_dict = {'f': 1, 'x': w, 'g': g}
  opts={'ipopt.tol': 1e-4,'ipopt.max_iter': 2000}
  # create solver from prob
  tic = time.time()
  solver = cs.nlpsol('solver', 'ipopt', prob_dict, opts)
  toc = time.time()
  print('time elapsed:', toc-tic)

  # ===========================================================
  # ===========================================================
  # ===========================================================

elif type_sol == 1:

  q = cs.SX.sym('q', n_q)
  q_dot = cs.SX.sym('q_dot', n_v)

  u = cs.SX.sym("actuated_torques", n_v - 6)
  tau = cs.vertcat(cs.SX.zeros(6, 1), u)

  # forces
  f_list = list()
  for i in range(n_c):
    f_list.append(cs.SX.sym(f'f{i}', n_f))

  contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
  contact_map = dict(zip(contacts_name, f_list))
  fd = kin_dyn.ForwardDynamics(kindyn, contacts_name, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

  q_ddot = fd.call(q, q_dot, tau, contact_map)

  x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)

  fs = cs.vertcat(*f_list)
  state = cs.vertcat(q, q_dot)
  input = cs.vertcat(u, fs)

  dae = {'x': state, 'p': input, 'ode': x_dot, 'quad': 1}
  integrator = RK4(dae, opts=dict(tf=0.01))

  g_list = list()
  w_list = list()

  int_map = integrator.map(N, 'thread', 4)

  input_list = list()
  x_list = list()
  for i in range(N):
    q_i = cs.SX.sym(f"q_{i}", n_q)
    q_dot_i = cs.SX.sym(f"q_dot_{i}", n_v)

    u_i = cs.SX.sym(f"u_{i}", n_v - 6)
    tau_i = cs.vertcat(cs.SX.zeros(6, 1), u_i)

    q_ddot_i = fd.call(q_i, q_dot_i, tau_i, contact_map)

    f_list_i = list()
    for k in range(n_c):
      f_list_i.append(cs.SX.sym(f'f{k}_{i}', n_f))

    fs_i = cs.vertcat(*f_list_i)
    input_i = cs.vertcat(u_i, fs_i)

    input_list.append(input_i)

    w_list.append(q_i)
    w_list.append(q_dot_i)
    w_list.append(u_i)
    w_list.append(fs_i)

    x_i, x_dot = utils.double_integrator_with_floating_base(q_i, q_dot_i, q_ddot_i)

    x_list.append(x_i)

  U = cs.horzcat(*input_list)
  X = cs.horzcat(*x_list)
  X_int = int_map(X, U)
  g = X_int[0] - X

  w = cs.vertcat(*w_list)
  # g = cs.vertcat(*g_list)

  print(w.shape)
  print(cs.vec(g).shape)

  prob_dict = {'f': 1, 'x': w, 'g': cs.vec(g)}
  opts = {'ipopt.tol': 1e-4, 'ipopt.max_iter': 2000}
  # create solver from prob
  tic = time.time()
  solver = cs.nlpsol('solver', 'ipopt', prob_dict, opts)
  toc = time.time()
  print('time elapsed:', toc - tic)

  # ===========================================================
  # ===========================================================
  # ===========================================================

elif type_sol == 2:

  q = cs.SX.sym('q', n_q)
  q_dot = cs.SX.sym('q_dot', n_v)

  u = cs.SX.sym("actuated_torques", n_v - 6)
  tau = cs.vertcat(cs.SX.zeros(6, 1), u)


  # forces
  f_list = list()
  for i in range(n_c):
      f_list.append(cs.SX.sym(f'f{i}', n_f))

  contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
  contact_map = dict(zip(contacts_name, f_list))
  fd = kin_dyn.ForwardDynamics(kindyn, contacts_name, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

  q_ddot = fd.call(q, q_dot, tau, contact_map)

  x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)

  fs = cs.vertcat(*f_list)
  state = cs.vertcat(q, q_dot)
  input = cs.vertcat(u, fs)

  dae = {'x': state, 'p': input, 'ode': x_dot, 'quad': 1}
  integrator = RK4(dae, opts=dict(tf=0.01))

  g_list = list()
  w_list = list()

  int_map = integrator.map(N, 'thread', 4)

  q_i = cs.MX.sym("q_i", n_q, N)
  q_dot_i = cs.MX.sym("q_dot_i", n_v, N)
  u_i = cs.MX.sym("u_i", n_v - 6, N)


  # f_list_i = list()
  # for k in range(n_c):
  #   f_list_i.append(cs.MX.sym(f'f{k}_i', n_f, N))
  #
  # f_i = cs.vertcat(*f_list_i)

  # all the forces
  f_i = cs.MX.sym('f_i', n_f*n_c, N)


  X = cs.vertcat(q_i, q_dot_i)
  U = cs.vertcat(u_i, f_i)

  X_int = int_map(X, U)
  g = X_int[0][1, :-1] - X[:,-2]

  print(g)

  w = cs.veccat(q_i, q_dot_i, u_i, f_i)

  # print(w.shape)
  # print(cs.vec(g).shape)
  # print(type(g))
  # print('diocane', g)
  # exit()

  prob_dict = {'f': 1, 'x': w, 'g': cs.vec(g)}
  opts={'ipopt.tol': 1e-4,'ipopt.max_iter': 2000}
  # create solver from prob
  tic = time.time()
  solver = cs.nlpsol('solver', 'ipopt', prob_dict, opts)
  toc = time.time()
  print('time elapsed:', toc-tic)

# elif type_sol == 3:

  # q = cs.SX.sym('q', n_q)
  # q_dot = cs.SX.sym('q_dot', n_v)
  # 
  # u = cs.SX.sym("actuated_torques", n_v - 6)
  # tau = cs.vertcat(cs.SX.zeros(6, 1), u)
  # 
  # 
  # # forces
  # f_list = list()
  # for i in range(n_c):
  #     f_list.append(cs.SX.sym(f'f{i}', n_f))
  # 
  # contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
  # contact_map = dict(zip(contacts_name, f_list))
  # fd = kin_dyn.ForwardDynamics(kindyn, contacts_name, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
  # 
  # q_ddot = fd.call(q, q_dot, tau, contact_map)
  # 
  # x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
  # 
  # fs = cs.vertcat(*f_list)
  # state = cs.vertcat(q, q_dot)
  # input = cs.vertcat(u, fs)
  # 
  # dae = {'x': state, 'p': input, 'ode': x_dot, 'quad': 1}
  # integrator = RK4(dae, opts=dict(tf=0.01))
  # 
  # g_list = list()
  # w_list = list()
  # 
  # 
  # 
  # 
  # # int_map = integrator.map(N, 'thread', 4)
  # 
  # q_i = cs.MX.sym("q_i", n_q, N)
  # q_dot_i = cs.MX.sym("q_dot_i", n_v, N)
  # u_i = cs.MX.sym("u_i", n_v, N)
  # u_i[:,0:6] = 0
  # 
  # # f_list_i = list()
  # # for k in range(n_c):
  # #   f_list_i.append(cs.MX.sym(f'f{k}_i', n_f, N))
  # #
  # # f_i = cs.vertcat(*f_list_i)
  # 
  # # all the forces
  # f_i = cs.MX.sym('f_i', n_f*n_c, N)
  # 
  # 
  # for i in range(N):
  # 
  #   x_prev_i = cs.vertcat(q_i[i,:], q_dot_i[i,:])
  # 
  #   q_ddot_i = fd.call(q_i[:, i], q_dot_i[:, i], u_i[:, i], contact_map)
  #   x_int = integrator(x0=x_prev_i, p=q_ddot_i)
  #   g_list.append(x_int["xf"] - x_prev_i)
  # 
  # w = cs.veccat(q_i, q_dot_i, u_i, f_i)
  # g = cs.vertcat(g_list)
  # 
  # # print(w.shape)
  # # print(cs.vec(g).shape)
  # # print(type(g))
  # # print('diocane', g)
  # # exit()
  # 
  # prob_dict = {'f': 1, 'x': w, 'g': cs.vec(g)}
  # opts={'ipopt.tol': 1e-4,'ipopt.max_iter': 2000}
  # # create solver from prob
  # tic = time.time()
  # solver = cs.nlpsol('solver', 'ipopt', prob_dict, opts)
  # toc = time.time()
  # print('time elapsed:', toc-tic)
# w0 = np.zeros(w.shape[0])
# lbw = -3 * np.ones(w.shape[0])
# ubw = 3 * np.ones(w.shape[0])
#
# lbg = - 100 * np.ones(2 * cs.vec(g).shape[0])
# ubg = 100 * np.ones(2 * cs.vec(g).shape[0])
#
#
# sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)



# contact_map = dict(zip(contacts_name, f_list_i))
# q_ddot_i_list = list()
# for i in range(N):
#   q_ddot_temp = fd.call(q_i[:, i], q_dot_i[:, i], tau_i[:, i], contact_map)
#   q_ddot_i_list.append(q_ddot_temp)
#
#
# q_ddot_i = cs.horzcat(*q_ddot_i_list)


# x_i_temp_list = list()
# x_dot_list = list()
# for i in range(N):
#   x_i_temp, x_dot_temp = utils.double_integrator_with_floating_base(q_i[:, i], q_dot_i[:, i], q_ddot_i[:, i])
#   x_i_temp_list.append(x_i_temp)
#   x_dot_list.append(x_dot_temp)
