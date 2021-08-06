from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon.utils import integrators, casadi_kin_dyn
import numpy as np
import casadi as cs

def resample_torques(p, v, a, node_time, dt, dae, frame_force_mapping, kindyn, force_reference_frame = cas_kin_dyn.CasadiKinDyn.LOCAL):
    """
        Resample solution to a different number of nodes, RK4 integrator is used for the resampling
        Args:
            p: position
            v: velocity
            a: acceleration
            node_time: previous node time
            dt: resampled period
            dae: a dictionary containing
                    'x': state
                    'p': control
                    'ode': a function of the state and control returning the derivative of the state
                    'quad': quadrature term
            frame_force_mapping: dictionary containing a map between frames and force variables e.g. {'lsole': F1}
            kindyn: object of type casadi_kin_dyn
            force_reference_frame: this is the frame which is used to compute the Jacobian during the ID computation:
                    LOCAL (default)
                    WORLD
                    LOCAL_WORLD_ALIGNED

        Returns:
            p_res: resampled p
            v_res: resampled v
            a_res: resampled a
            frame_res_force_mapping: resampled frame_force_mapping
            tau_res: resampled tau
        """

    p_res, v_res, a_res = second_order_resample_integrator(p, v, a, node_time, dt, dae)

    frame_res_force_mapping = dict()
    for frame, wrench in frame_force_mapping.items():
        frame_res_force_mapping[frame] = resample_input(wrench, node_time, dt)
    tau_res = np.zeros(a_res.shape)

    ID = casadi_kin_dyn.InverseDynamics(kindyn, frame_force_mapping.keys(), force_reference_frame)
    ni = a_res.shape[1]
    for i in range(ni):
        frame_force_map_i = dict()
        for frame, wrench in frame_res_force_mapping.items():
            frame_force_map_i[frame] = wrench[:, i]
        tau_i = ID.call(p_res[:, i], v_res[:, i], a_res[:, i], frame_force_map_i)
        tau_res[:, i] = tau_i.toarray().flatten()

    return p_res, v_res, a_res, frame_res_force_mapping, tau_res

def resample_input(input, node_time, dt):
    """
    Resample an input variable according to a new sample time dt.
    NOTE: the resampling is done by considering constant input between nodes
    Args:
        input: input to resample
        node_time: original node time
        dt: new node time
    Returns:
        input_res: resampled input
    """
    number_of_nodes = input.shape[1]+1
    node_time_array = np.zeros([number_of_nodes])
    if hasattr(node_time, "__iter__"):
        for i in range(1, number_of_nodes):
            node_time_array[i] = node_time_array[i - 1] + node_time[i - 1]
    else:
        for i in range(1, number_of_nodes):
            node_time_array[i] = node_time_array[i - 1] + node_time

    n_res = int(round(node_time_array[-1] / dt))

    input_res = np.zeros([input.shape[0], n_res])

    t = 0.
    node = 0
    i = 0
    while i < input_res.shape[1] - 1:
        input_res[:, i] = input[:, node]
        t += dt
        i += 1
        if t > node_time_array[node + 1]:
            node += 1

    return input_res


def second_order_resample_integrator(p, v, u, node_time, dt, dae):
    """
    Resample a solution with the given dt (RK4 integrator is used internally)
    Args:
        p: position
        v: velocity
        u: input
        node_time: previous node time
        dt: resampling time
        dae: dynamic model
    Returns:
        p_res: resampled position
        v_res: resampled velocity
        u_res: resampled input
    """
    number_of_nodes = p.shape[1]
    node_time_array = np.zeros([number_of_nodes])
    if hasattr(node_time, "__iter__"):
        for i in range(1, number_of_nodes):
            node_time_array[i] = node_time_array[i-1] + node_time[i - 1]
    else:
        for i in range(1, number_of_nodes):
            node_time_array[i] = node_time_array[i-1] + node_time

    n_res = int(round(node_time_array[-1]/dt))

    opts = {'tf': dt}
    F_integrator = integrators.RK4(dae, opts, cs.SX)

    x_res0 = np.hstack((p[:, 0], v[:, 0]))

    x_res = np.zeros([p.shape[0] + v.shape[0], n_res+1])
    p_res = np.zeros([p.shape[0], n_res+1])
    v_res = np.zeros([v.shape[0], n_res+1])
    u_res = np.zeros([u.shape[0], n_res])

    x_res[:, 0] = x_res0
    p_res[:, 0] = x_res0[0:p.shape[0]]
    v_res[:, 0] = x_res0[p.shape[0]:]
    u_res[:, 0] = u[:, 0]

    t = 0.
    i = 0
    node = 0
    while i < u_res.shape[1]-1:
        x_resi = F_integrator(x0=x_res[:, i], p=u[:, node])['xf'].toarray().flatten()

        t += dt
        i += 1

        #print(f"{t} <= {tf-dt} @ node time {(node+1)*node_time} i: {i}")

        x_res[:, i] = x_resi
        p_res[:, i] = x_resi[0:p.shape[0]]
        v_res[:, i] = x_resi[p.shape[0]:]
        u_res[:, i] = u[:, node]

        if t > node_time_array[node+1]:
            new_dt = t - node_time_array[node+1]
            node += 1
            if new_dt >= 1e-6:
                opts = {'tf': new_dt}
                new_F_integrator = integrators.RK4(dae, opts, cs.SX)
                x_resi = new_F_integrator(x0=np.hstack((p[:,node], v[:,node])), p=u[:, node])['xf'].toarray().flatten()
                x_res[:, i] = x_resi
                p_res[:, i] = x_resi[0:p.shape[0]]
                v_res[:, i] = x_resi[p.shape[0]:]
                u_res[:, i] = u[:, node]

    x_resf = np.hstack((p[:, -1], v[:, -1]))
    x_res[:, -1] = x_resf
    p_res[:, -1] = x_resf[0:p.shape[0]]
    v_res[:, -1] = x_resf[p.shape[0]:]

    return p_res, v_res, u_res


