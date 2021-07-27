from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon.utils import integrators, casadi_kin_dyn
import numpy as np
import casadi as cs

def resample_torques(p, v, a, tf, dt, dae, frame_force_mapping, kindyn, force_reference_frame = cas_kin_dyn.CasadiKinDyn.LOCAL):
    """
        Resample solution to a different number of nodes, RK4 integrator is used for the resampling
        Args:
            X: state variable
            U_integrator: control variable
            time: the final time (tf) or the vector of intermediate periods (dt_hist)
            dt: resampled period
            dae: a dictionary containing
                    'x': state
                    'p': control
                    'ode': a function of the state and control returning the derivative of the state
                    'quad': quadrature term
            ID: Function.deserialize(kindyn.rnea()) TODO: remove, this can be taken from kindyn
            dict: dictionary containing a map between frames and force variables e.g. {'lsole': F1}
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

    p_res, v_res, a_res = second_order_resample_integrator(p, v, a, tf, dt, dae)
    ni = a_res.shape[1]
    frame_res_force_mapping = dict()
    for key in frame_force_mapping:
        frame_res_force_mapping[key] = np.zeros([frame_force_mapping[key].shape[0], ni])
    t = 0.
    node = 0
    i = 0
    number_of_nodes = p.shape[1]
    node_time = tf / (number_of_nodes - 1)
    while t <= tf:
        for key in frame_force_mapping:
            frame_res_force_mapping[key][:, i] = frame_force_mapping[key][:, node]
        t += dt
        i += 1
        if t > (node + 1) * node_time:
            node += 1

    tau_res = np.zeros(a_res.shape)

    ID = casadi_kin_dyn.InverseDynamics(kindyn, frame_force_mapping.keys())
    for i in range(ni):
        frame_force_map_i = dict()
        for frame, wrench in frame_res_force_mapping.items():
            frame_force_map_i[frame] = wrench[:, i]
        tau_i = ID.call(p_res[:, i], v_res[:, i], a_res[:, i], frame_force_map_i)
        tau_res[:, i] = tau_i.toarray().flatten()

    return p_res, v_res, a_res, frame_res_force_mapping, tau_res

def second_order_resample_integrator(p, v, a, tf, dt, dae):
    """
    Resample a solution with the given dt
    Args:
        p: position
        v: velocity
        a: acceleration
        tf: final time
        dt: resampling time
        dae: dynamic model
    Return:
        p_res: resampled position
        v_res: resampled velocity
        a_res: resampled acceleration
    """
    number_of_nodes = p.shape[1]
    node_time = tf/(number_of_nodes-1)
    n_res = int(round(tf/dt))

    opts = {'tf': dt}
    F_integrator = integrators.RK4(dae, opts, 'SX')

    x_res0 = np.hstack((p[:, 0], v[:, 0]))

    x_res = np.zeros([p.shape[0] + v.shape[0], n_res+1])
    p_res = np.zeros([p.shape[0], n_res+1])
    v_res = np.zeros([v.shape[0], n_res+1])
    a_res = np.zeros([a.shape[0], n_res])

    x_res[:, 0] = x_res0
    p_res[:, 0] = x_res0[0:p.shape[0]]
    v_res[:, 0] = x_res0[p.shape[0]:]
    a_res[:, 0] = a[:, 0]

    t = 0.
    i = 0
    node = 0
    while t <= tf-dt:
        x_resi = F_integrator(x0=x_res[:, i], p=a[:, node])['xf'].toarray().flatten()

        t += dt
        i += 1

        x_res[:, i] = x_resi
        p_res[:, i] = x_resi[0:p.shape[0]]
        v_res[:, i] = x_resi[p.shape[0]:]
        a_res[:, i] = a[:, node]

        if t > (node+1)*node_time:
            new_dt = t - (node + 1) * node_time
            node += 1
            if new_dt >= 1e-6:
                opts = {'tf': new_dt}
                new_F_integrator = integrators.RK4(dae, opts, 'SX')
                x_resi = new_F_integrator(x0=np.hstack((p[:,node], v[:,node])), p=a[:, node])['xf'].toarray().flatten()
                x_res[:, i] = x_resi
                p_res[:, i] = x_resi[0:p.shape[0]]
                v_res[:, i] = x_resi[p.shape[0]:]
                a_res[:, i] = a[:, node]

    x_resf = np.hstack((p[:, -1], v[:, -1]))
    x_res[:, -1] = x_resf
    p_res[:, -1] = x_resf[0:p.shape[0]]
    v_res[:, -1] = x_resf[p.shape[0]:]

    return p_res, v_res, a_res


