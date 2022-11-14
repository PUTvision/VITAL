#!/usr/bin/env python3
import numpy as np
import rospy
import rospkg

from mavros_msgs.msg import Altitude
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from visual_landing_provider.utils.ros import parse_ros_params, generate_img_msg, generate_pose_msg
from visual_landing_provider.visual_pipeline_wrapper import VisualPipelineWrapper


class VisualInference:
    def __init__(self, params: dict):
        """ Visual Inference node.

        Parameters
        ----------
        params : dict
            Parameters of the node.
        """
        rospy.init_node('visual_infer_node', anonymous=True)

        self.altitude = None

        self.vis_infer = params['/vis_infer']
        self.base_path = rospkg.RosPack().get_path('visual_landing_provider')

        self.pipeline = VisualPipelineWrapper(base_path=self.base_path, config=params, visualize=self.vis_infer)

        self.pose_publisher = rospy.Publisher("/visual_infer/pose", Pose, queue_size=10)

        if self.vis_infer:
            self.image_publisher = rospy.Publisher("/visual_infer/image_raw", Image, queue_size=10)

        rospy.Subscriber(params['/image_topic'], Image, self._image_callback)
        rospy.Subscriber(params['/mavros_altitude_topic'], Altitude, self._altitude_callback)

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
            self.pose_publisher.publish(generate_pose_msg(results['pose']))
        else:
            rospy.loginfo(f'Error: {results["error"]}')

        if self.vis_infer and results['image'] is not None:
            self.image_publisher.publish(generate_img_msg(results['image']))

if __name__ == '__main__':
    params = parse_ros_params()

    vi = VisualInference(params=params)
    rospy.spin()
