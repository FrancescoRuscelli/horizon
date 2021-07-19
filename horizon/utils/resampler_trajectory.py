import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
from horizon.utils import integrators
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf as ros_tf
import geometry_msgs.msg
import time

def normalize_quaternion(q):

    def normalize(v):
        return v / np.linalg.norm(v)

    quat = normalize([q[3], q[4], q[5], q[6]])
    q[3:7] = quat[0:4]

    return q


class replay_trajectory:
    def __init__(self, dt, joint_list, q_replay):
        """
        Contructor
        Args:
            dt:
            joint_list:
            q_replay:
        """
        self.dt = dt
        self.joint_list = joint_list
        self.q_replay = q_replay
        self.__sleep = 0.0

    def sleep(self, secs):
        '''
        Set sleep time between trajectory sequences
        Args:
            secs: time to sleep in seconds
        '''
        self.__sleep = secs

    def replay(self):
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher')
        rate = rospy.Rate(1. / self.dt)
        joint_state_pub = JointState()
        joint_state_pub.header = Header()
        joint_state_pub.name = self.joint_list

        br = ros_tf.TransformBroadcaster()
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'world'
        m.child_frame_id = 'base_link'

        nq = np.shape(self.q_replay)[1]

        while not rospy.is_shutdown():
            for qk in self.q_replay.T:
                qk = normalize_quaternion(qk)

                m.transform.translation.x = qk[0]
                m.transform.translation.y = qk[1]
                m.transform.translation.z = qk[2]
                m.transform.rotation.x = qk[3]
                m.transform.rotation.y = qk[4]
                m.transform.rotation.z = qk[5]
                m.transform.rotation.w = qk[6]

                br.sendTransform((m.transform.translation.x, m.transform.translation.y, m.transform.translation.z),
                                 (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
                                  m.transform.rotation.w),
                                 rospy.Time.now(), m.child_frame_id, m.header.frame_id)

                t = rospy.Time.now()
                joint_state_pub.header.stamp = t
                joint_state_pub.position = qk[7:nq]
                joint_state_pub.velocity = []
                joint_state_pub.effort = []
                pub.publish(joint_state_pub)
                rate.sleep()
            if self.__sleep > 0.:
                time.sleep(self.__sleep)


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
        X_res: resampled state
        Tau_res: resampled torques
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


