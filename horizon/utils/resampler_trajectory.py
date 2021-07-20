import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
from horizon.utils import integrators
import numpy as np


def second_order_resample_integrator(p, v, a, tf, dt, dae, frame_force_mapping, kindyn, force_reference_frame = cas_kin_dyn.CasadiKinDyn.LOCAL):
    """
    Resample solution to a different number of nodes, RK4 integrator is used for the resampling
    Args:
        p: position values
        v: velocity values
        a: acceleration values
        tf: the final time (tf) or the vector of intermediate periods (dt_hist)
        dt: resampled period
        dae: a dictionary containing
                'x': state
                'p': control
                'ode': a function of the state and control returning the derivative of the state
                'quad': quadrature term
        frame_force_mapping: dictionary containing a map between frames and force variables e.g. {'lsole': f1}
        kindyn: object of type casadi_kin_dyn
        force_reference_frame: this is the frame which is used to compute the Jacobian during the ID computation:
                LOCAL (default)
                WORLD
                LOCAL_WORLD_ALIGNED

    Returns:
        p_res: resampled p
        v_res: resampled v
        a_res: resampled a
        TODO: add resampled controls!
    """
    ns = p.shape[1]

    if v.shape[1] != ns:
        raise Exception("length of state: {} != lenght of state ({})".format(v.shape[1], ns))
    if a.shape[1] != ns - 1:
        raise Exception("length of input: {} != lenght of state -1 ({})".format(a.shape[1], ns - 1))

    ti = tf / (ns - 1)  # interval time

    if dt >= ti:
        dt = ti
        ni = 1
    else:
        ni = int(round(ti / dt))  # number of intermediate nodes in interval

    opts = {'tf': dt}
    F_integrator = integrators.RK4(dae, opts, 'SX')

    n_res = (ns - 1) * ni

    x_res0 = np.hstack((p[:,0], v[:,0]))

    x_res = np.zeros([p.shape[0] + v.shape[0], n_res + 1])
    p_res = np.zeros([p.shape[0], n_res + 1])
    v_res = np.zeros([v.shape[0], n_res + 1])
    a_res = np.zeros([a.shape[0], n_res])

    x_res[:, 0] = x_res0
    p_res[:, 0] = x_res0[0:p.shape[0]]
    v_res[:, 0] = x_res0[p.shape[0]:]
    a_res[:, 0] = a[:, 0]

    k = 0
    for i in range(0, ns-1):  # cycle on intervals
        for j in range(0, ni):  # cycle on intermediate nodes in interval
            x_resi=None
            if j == 0:
                x_resi = F_integrator(x0=np.hstack((p[:,i], v[:,i])), p=a[:,i])['xf'].toarray().flatten()
            else:
                x_resi = F_integrator(x0=x_res[:, k], p=a[:,i])['xf'].toarray().flatten()

            x_res[:, k+1] = x_resi
            p_res[:, k+1] = x_resi[0:p.shape[0]]
            v_res[:, k+1] = x_resi[p.shape[0]:]
            a_res[:, 0] = a[:, i]
            k += 1

    x_resf = np.hstack((p[:, -1], v[:, -1]))

    x_res[:, -1] = x_resf
    p_res[:, -1] = x_resf[0:p.shape[0]]
    v_res[:, -1] = x_resf[p.shape[0]:]

    return p_res, v_res, a_res


