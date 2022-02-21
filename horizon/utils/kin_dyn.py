from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np

def surface_point_contact(plane_dict, q, kindyn, frame):
    """
    Position ONLY constraint to lies into a plane: ax + by + cz +d = 0
    todo:Add orientation as well
    Args:
        plane_dict: which contains following variables:
                        - a
                        - b
                        - c
                        - d
                    to define the plane ax + by + cz +d = 0
        q: position state variables
        kindyn: casadi_kin_dyn object
        frame: name of the point frame constrained in the plane
    Returns:
        constraint
        lb
        ub
    """
    P = np.array([0., 0., 0.])
    d = 0.

    if 'a' in plane_dict:
        P[0] = plane_dict['a']
    if 'b' in plane_dict:
        P[1] = plane_dict['b']
    if 'c' in plane_dict:
        P[2] = plane_dict['c']

    if 'd' in plane_dict:
        d = plane_dict['d']

    FK = cs.Function.deserialize(kindyn.fk(frame))
    CLink_pos = FK(q=q)['ee_pos']

    constraint = cs.dot(P, CLink_pos)
    lb = -d
    ub = -d
    return constraint, lb, ub


def linearized_friction_cone(f, mu, R):
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

def linearized_friction_cone_map(f, mu, R, N):
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
    a = cs.MX.sym('a_fr_r', 5, 3)
    b = cs.MX.sym('w', 3)
    fc = cs.Function('fc', [a, b], [cs.mtimes(a, b)])
    fc = fc.map(N)

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
    res = fc(A_fr_R, f)

    # lb = np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf])
    # ub = np.array([0., 0., 0., 0., 0.])

    return res #, lb, ub


class ForwardDynamics():
    """
    Class which computes forward dynamics:
    given generalized position, velocities, torques and contact forces, returns generalized accelerations
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
        self.fd = kindyn.aba()
        self.contact_jacobians = dict()
        for frame in contact_frames:
            self.contact_jacobians[frame] = cs.Function.deserialize(kindyn.jacobian(frame, force_reference_frame))

    def call(self, q, qdot, tau, frame_force_mapping=dict()):
        """
                Computes generalized accelerations:
                Args:
                    q: joint positions
                    qdot: joint velocities
                    torques: joint torques
                    frame_force_mapping: dictionary containing a map between frames and force variables e.g. {'lsole': F1} representing the frame
                        where the force is acting (the force is expressed in force_reference_frame!)
                Returns:
                    qddot: generalized accelerations
                """
        JtF_sum = 0

        for frame, wrench in frame_force_mapping.items():
            J = self.contact_jacobians[frame](q=q)['J']

            if wrench.shape[0] == 3:  # point contact
                JtF = cs.mtimes(J[0:3, :].T, wrench)
            else:  # surface contact
                JtF = cs.mtimes(J.T, wrench)
            JtF_sum += JtF

        qddot = self.fd(q=q, v=qdot, tau=tau + JtF_sum)['a']
        return qddot

class InverseDynamics():
    """
    Class which computes inverse dynamics:
    given generalized position, velocities, accelerations and contact forces, returns generalized torques
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

    def call(self, q, qdot, qddot, frame_force_mapping=dict(), tau_ext=0):
        """
        Computes generalized torques:
        Args:
            q: joint positions
            qdot: joint velocities
            qddot: joint accelerations
            frame_force_mapping: dictionary containing a map between frames and force variables e.g. {'lsole': F1} representing the frame
                where the force is acting (the force is expressed in force_reference_frame!)
            tau_ext: extra torques input for dynamics
        Returns:
            tau: generalized torques
        """
        JtF_sum = 0
        for frame, wrench in frame_force_mapping.items():
            J = self.contact_jacobians[frame](q=q)['J']
            if wrench.shape[0] == 3:  # point contact
                JtF = cs.mtimes(J[0:3, :].T, wrench)
            else:  # surface contact
                JtF = cs.mtimes(J.T, wrench)
            JtF_sum += JtF

        tau = self.id(q=q, v=qdot, a=qddot)['tau'] - JtF_sum - tau_ext
        return tau


class InverseDynamicsMap():
    """
    Class which computes inverse dynamics:
    given generalized position, velocities, accelerations and contact forces, returns generalized torques
    """

    def __init__(self, N, kindyn, contact_frames=[], force_reference_frame=cas_kin_dyn.CasadiKinDyn.LOCAL):
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
        self.id = self.id.map(N, 'thread', 15)

        J_point = cs.MX.sym('Jac', 3, kindyn.nv())
        w_point = cs.MX.sym('w', 3)
        self.JtF_fun_point = cs.Function('JtF_fun', [J_point, w_point], [cs.mtimes(J_point.T, w_point)])
        self.JtF_fun_point = self.JtF_fun_point.map(N, 'thread', 15)

        J_planar = cs.MX.sym('Jac', 3, kindyn.nv())
        w_planar = cs.MX.sym('w', 3)
        self.JtF_fun_planar = cs.Function('JtF_fun', [J_planar, w_planar], [cs.mtimes(J_planar.T, w_planar)])
        self.JtF_fun_planar = self.JtF_fun_planar.map(N, 'thread', 15)

        self.contact_jacobians = dict()
        for frame in contact_frames:
            self.contact_jacobians[frame] = cs.Function.deserialize(kindyn.jacobian(frame, force_reference_frame))
            self.contact_jacobians[frame] = self.contact_jacobians[frame].map(N, 'thread', 15)

    def call(self, q, qdot, qddot, frame_force_mapping = dict()):
        """
        Computes generalized torques:
        Args:
            q: joint positions
            qdot: joint velocities
            qddot: joint accelerations
            frame_force_mapping: dictionary containing a map between frames and force variables e.g. {'lsole': F1} representing the frame
                where the force is acting (the force is expressed in force_reference_frame!)
        Returns:
            tau: generalized torques
        """
        JtF_sum = 0

        for frame, wrench in frame_force_mapping.items():
            # matrices are aggregated horizontally
            J = self.contact_jacobians[frame](q=q)['J']

            if wrench.shape[0] == 3:  # point contact
                JtF = self.JtF_fun_point(J[0:3, :], wrench)
            else:  # surface contact
                JtF = self.JtF_fun_planar(J, wrench)
            JtF_sum += JtF


        tau = self.id(q=q, v=qdot, a=qddot)['tau'] - JtF_sum
        return tau







