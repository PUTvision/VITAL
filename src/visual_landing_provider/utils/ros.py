import numpy as np
import rospy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler


def parse_ros_params():
    """Parse ROS parameters.

    Returns:
        dict: Parsed ROS parameters.
    """
    params = {}
    for key in rospy.get_param_names():
        params[key] = rospy.get_param(key)
        
    return params

def generate_pose_msg(results: np.ndarray) -> Pose:
    """ Generate pose message.

    Parameters
    ----------
    visual : np.ndarray
        Translation and rotation of the landing target (6,).

    Returns
    -------
    Pose
        Pose message.
    """
    msg = Pose()
    msg.position.x = results[0]
    msg.position.y = results[1]
    msg.position.z = results[2]

    q = quaternion_from_euler(results[3], results[4], results[5])

    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]

    return msg


def generate_img_msg(visual: np.ndarray) -> Image:
    """ Generate image message.

    Parameters
    ----------
    visual : np.ndarray
        Visualized image in RGB order.

    Returns
    -------
    Image
        Image message.
    """
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = visual.shape[0]
    msg.width = visual.shape[1]
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = 3 * visual.shape[1]
    msg.data = np.array(visual).tobytes()

    return msg
