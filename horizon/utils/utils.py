import casadi as cs

def jac(dict, var_string_list, function_string_list):
    """
    Args:
        dict: dictionary which maps variables and functions, eg. {'x': x, 'u': u, 'f': f}
        var_string_list: list of variables in dict, eg. ['x', 'u']
        function_string_list: list of functions in dict, eg. ['f']
    Returns:
        F: casadi Function for evaluation
        jac: dictionary with expression of derivatives

    NOTE: check /tests/jac_test.py for example of usage for Jacobian and Hessian computation
    """
    f = {}
    for function in function_string_list:
        f[function] = dict[function]

    vars_dict = {}
    X = []
    for var in var_string_list:
        vars_dict[var] = dict[var]
        X.append(dict[var])

    jac_list = []
    jac_id_list = []
    for function_key in f:
        for var in var_string_list:
            id = "D" + function_key + 'D' + var
            jac_id_list.append(id)
            jac_list.append(cs.jacobian(f[function_key], vars_dict[var]))

    jac_map = {}
    i = 0
    for jac_id in jac_id_list:
        jac_map[jac_id] = jac_list[i]
        i += 1

    F = cs.Function('jacobian', X, jac_list, var_string_list, jac_id_list)

    return F, jac_map


def skew(q):
    """
    Create skew matrix from vector part of quaternion
    Args:
        q: vector part of quaternion [qx, qy, qz]

    Returns:
        S = skew symmetric matrix built using q
    """
    S = cs.SX.zeros(3, 3)
    S[0, 1] = -q[2]; S[0, 2] = q[1]
    S[1, 0] = q[2];  S[1, 2] = -q[0]
    S[2, 0] = -q[1]; S[2, 1] = q[0]
    return S

def quaterion_product(q, p):
    """
    Computes quaternion product between two quaternions q and p
    Args:
        q: quaternion
        p: quaternion

    Returns:
        quaternion product q x p
    """
    q0 = q[3]
    p0 = p[3]

    return [q0*p[0:3] + p0*q[0:3] + cs.mtimes(skew(q[0:3]), p[0:3]), q0*p0 - cs.mtimes(q[0:3].T, p[0:3])]

def toRot(q):
    """
    Compute rotation matrix associated to given quaternion q
    Args:
        q: quaternion

    Returns:
        R: rotation matrix

    """
    R = cs.SX.zeros(3, 3)
    qi = q[0]; qj = q[1]; qk = q[2]; qr = q[3]
    R[0, 0] = 1. - 2. * (qj * qj + qk * qk);
    R[0, 1] = 2. * (qi * qj - qk * qr);
    R[0, 2] = 2. * (qi * qk + qj * qr)
    R[1, 0] = 2. * (qi * qj + qk * qr);
    R[1, 1] = 1. - 2. * (qi * qi + qk * qk);
    R[1, 2] = 2. * (qj * qk - qi * qr)
    R[2, 0] = 2. * (qi * qk - qj * qr);
    R[2, 1] = 2. * (qj * qk + qi * qr);
    R[2, 2] = 1. - 2. * (qi * qi + qj * qj)

    return R


def double_integrator_with_floating_base(q, ndot, nddot):
    """
    Construct the floating-base dynamic model:
                x = [q, ndot]
                xdot = [qdot, nddot]
    using quaternion dynamics: quatdot = quat x [omega, 0]
    NOTE: this implementation consider floating-base position and orientation expressed in GLOBAL (world) coordinates while
    linear and angular velocities expressed in LOCAL (base_link) coordinates.
    Args:
        q: joint space coordinates: q = [x y z px py pz pw qj], where p is a quaternion
        ndot: joint space velocities: ndot = [vx vy vz wx wy wz qdotj]
        nddot: joint space acceleration: nddot = [ax ay ax wdotx wdoty wdotz qddotj]

    Returns:
        x: state x = [q, ndot]
        xdot: derivative of the state xdot = [qdot, nddot]
    """

    qw = cs.SX.zeros(4,1)
    qw[0:3] = 0.5*ndot[3:6]
    quaterniondot = quaterion_product(q[3:7], qw)

    R = toRot(q[3:7])

    x = cs.vertcat(q, ndot)
    xdot = cs.vertcat(cs.mtimes(R, ndot[0:3]), cs.vertcat(*quaterniondot), ndot[6:ndot.shape[0]], nddot)

    return x, xdot


def double_integrator(q, qdot, qddot):
    x = cs.vertcat(q, qdot)
    xdot = cs.vertcat(qdot, qddot)
    return x, xdot

