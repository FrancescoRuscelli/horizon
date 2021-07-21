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

    def replay(self, is_floating_base=True):
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher')
        rate = rospy.Rate(1. / self.dt)
        joint_state_pub = JointState()
        joint_state_pub.header = Header()
        joint_state_pub.name = self.joint_list

        if is_floating_base:
            br = ros_tf.TransformBroadcaster()
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = 'world'
            m.child_frame_id = 'base_link'

        nq = np.shape(self.q_replay)[1]

        while not rospy.is_shutdown():
            for qk in self.q_replay.T:

                if is_floating_base:
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
                joint_state_pub.position = qk[7:nq] if is_floating_base else qk
                joint_state_pub.velocity = []
                joint_state_pub.effort = []
                pub.publish(joint_state_pub)
                rate.sleep()
            if self.__sleep > 0.:
                time.sleep(self.__sleep)