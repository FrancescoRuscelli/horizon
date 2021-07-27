from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np

def linearized_friciton_cone(f, mu, R):
    """
    Args:
        f: force (only linear components)
        mu: friciton cone
        R: rotation matrix between force reference frame and surface frame (ff_R_cf, ff: force frame, cf: contact frame)

    Returns:
        constraint
        lb
        ub
    """

    mu_lin = mu / 2.0 * np.sqrt(2.0)

    A_fr = np.zeros([5, 3])
    A_fr[0, 0] = 1.0
    A_fr[0, 2] = -mu_lin
    A_fr[1, 0] = -1.0
    A_fr[1, 2] = -mu_lin
    A_fr[2, 1] = 1.0
    A_fr[2, 2] = -mu_lin
    A_fr[3, 1] = -1.0
    A_fr[3, 2] = -mu_lin
    A_fr[4, 2] = -1.0

    A_fr_R = cs.mtimes(A_fr, R.T)

    return cs.mtimes(A_fr_R, f), [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf], [0., 0., 0., 0., 0.]

class InverseDynamics():
    """
    Class which computes inverse dynamics:
    given generalized position, velocities and accelerations returns generalized torques
    """
    def __init__(self, kindyn, contact_frames = [], force_reference_frame = cas_kin_dyn.CasadiKinDyn.LOCAL):
        """
        Args:
            kindyn: casadi_kin_dyn object
            contact_frames: list of contact frames
            force_reference_frame: this is the frame which is used to compute the Jacobian during the ID computation:
                LOCAL (default)
                WORLD
                LOCAL_WORLD_ALIGNED
        """
        self.id = cs.Function.deserialize(kindyn.rnea())
        self.contact_jacobians = dict()
        for frame in contact_frames:
            self.contact_jacobians[frame] = cs.Function.deserialize(kindyn.jacobian(frame, force_reference_frame))

    def call(self, q, qdot, qddot, frame_force_mapping = dict()):
        """
        Computes generalized torques:
        Args:
            q: joint positions
            qdot: joint velocities
            qddot: joint accelerations
            frame_force_mapping: dictionary containing a map between frames and force variables e.g. {'lsole': F1} representing the frame
                where the force is acting (the force is expressed in force_reference_frame!)
        """
        JtF_sum = 0

        for frame, wrench in frame_force_mapping.items():
            J = self.contact_jacobians[frame](q=q)['J']
            if wrench.shape[0] == 3:  # point contact
                JtF = cs.mtimes(J[0:3, :].T, wrench)
            else:  # surface contact
                JtF = cs.mtimes(J.T, wrench)
            JtF_sum += JtF

        tau = self.id(q=q, v=qdot, a=qddot)['tau'] - JtF_sum
        return tau








