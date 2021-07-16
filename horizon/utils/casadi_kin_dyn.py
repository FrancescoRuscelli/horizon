import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs

def inverse_dynamics(q, qdot, qddot, frame_force_mapping, kindyn, force_reference_frame = cas_kin_dyn.CasadiKinDyn.LOCAL):
    """
    Class which computes inverse dynamics:
    given generalized position, velocities and accelerations returns generalized torques

    Args:
        q: joint positions
        qdot: joint velocities
        qddot: joint accelerations
        frame_force_mapping: dictionary containing a map between frames and force variables e.g. {'lsole': F1}
        kindyn: casadi_kin_dyn object
        force_reference_frame: this is the frame which is used to compute the Jacobian during the ID computation:
            LOCAL (default)
            WORLD
            LOCAL_WORLD_ALIGNED
    """

    JtF_sum = None
    if issubclass(type(q), cs.SX):
        JtF_sum = cs.SX([0])
    elif issubclass(type(q), cs.MX):
        JtF_sum = cs.MX([0])

    for frame, wrench in frame_force_mapping.items():
        Jac = cs.Function.deserialize(kindyn.jacobian(frame, force_reference_frame))
        J = Jac(q=q)['J']
        if(wrench.shape[0] == 3): # point contact
            JtF = cs.mtimes(J[0:3,:].T, wrench)
        else: # surface contact
            JtF = cs.mtimes(J.T, wrench)
        JtF_sum += JtF

    id = cs.Function.deserialize(kindyn.rnea())
    tau = id(q=q, v=qdot, a=qddot)['tau'] - JtF_sum
    return tau






