import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TransformBroadcaster:
    def __init__(self) -> None:
        self.pub = rospy.Publisher(name='/tf', data_class=TFMessage, queue_size=10)

    def sendTransform(self, pos, rot, time, child_frame_id, frame_id):
        tfmsg = TFMessage()
        transf = TransformStamped()
        transf.header.stamp = time
        transf.header.frame_id = frame_id
        transf.child_frame_id = child_frame_id
        transf.transform.translation.x = pos[0]
        transf.transform.translation.y = pos[1]
        transf.transform.translation.z = pos[2]
        transf.transform.rotation.x = rot[0]
        transf.transform.rotation.y = rot[1]
        transf.transform.rotation.z = rot[2]
        transf.transform.rotation.w = rot[3]
        tfmsg.transforms.append(transf)
        self.pub.publish(tfmsg)