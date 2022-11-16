#!/usr/bin/env python3
import numpy as np
import rospy
import rospkg
from tf.transformations import quaternion_from_euler

from mavros_msgs.msg import Altitude
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from visual_landing_provider.vital.visual_pipeline_wrapper import VisualPipelineWrapper



class VisualInference:
    def __init__(self):
        """ Visual Inference node.

        """
        rospy.init_node('visual_infer_node', anonymous=True)

        self.altitude = None

        self.params = self.parse_ros_params()

        self.vis_infer = self.params['/vis_infer']
        self.base_path = rospkg.RosPack().get_path('visual_landing_provider')

        self.pipeline = VisualPipelineWrapper(base_path=self.base_path, config=self.params, visualize=self.vis_infer)

        self.pose_publisher = rospy.Publisher("/visual_infer/pose", Pose, queue_size=10)

        if self.vis_infer:
            self.image_publisher = rospy.Publisher("/visual_infer/image_raw", Image, queue_size=10)

        rospy.Subscriber(self.params['/image_topic'], Image, self._image_callback)
        rospy.Subscriber(self.params['/mavros_altitude_topic'], Altitude, self._altitude_callback)

        rospy.loginfo(f'Node visual_infer_node started!')

    def _altitude_callback(self, data: Altitude):
        """ Altitude callback.

        Parameters
        ----------
        data : Altitude
            Altitude message.
        """
        self.altitude = data.local

    def _image_callback(self, data: Image):
        """ Image callback. Main loop of the node.

        Parameters
        ----------
        data : Image
            Image message.
        """
        if self.altitude is None:
            rospy.loginfo('Altitude is not estimated')
            return

        image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        results = self.pipeline.process(image, altitude=self.altitude)

        if results['pose'] is not None:
            self.pose_publisher.publish(self.generate_pose_msg(results['pose']))
        else:
            rospy.loginfo(f'Error: {results["error"]}')

        if self.vis_infer and results['image'] is not None:
            self.image_publisher.publish(self.generate_img_msg(results['image']))

    def parse_ros_params(self):
        """Parse ROS parameters.

        Returns:
            dict: Parsed ROS parameters.
        """
        params = {}
        for key in rospy.get_param_names():
            params[key] = rospy.get_param(key)
            
        return params

    def generate_pose_msg(self, results: np.ndarray) -> Pose:
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


    def generate_img_msg(self, visual: np.ndarray) -> Image:
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


if __name__ == '__main__':
    vi = VisualInference()
    rospy.spin()
