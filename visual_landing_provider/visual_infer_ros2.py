#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
import rospkg
from tf_transformations import quaternion_from_euler

from mavros_msgs.msg import Altitude
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from visual_landing_provider.vital.visual_pipeline_wrapper import VisualPipelineWrapper
from visual_landing_provider.vital.utils.common import parse_yaml_params 

class VisualInference(Node):
    def __init__(self):
        """ Visual Inference node.

        Parameters
        ----------
        """
        super().__init__('visual_infer_node', allow_undeclared_parameters=True,
                       automatically_declare_parameters_from_overrides=True)

        self.altitude = None
        self.base_path = get_package_share_directory('visual_landing_provider')

        self.params = parse_yaml_params(self.base_path)

        print(self.params)

        self.vis_infer = self.params['/vis_infer']

        self.pipeline = VisualPipelineWrapper(base_path=self.base_path, config=self.params, visualize=self.vis_infer)

        self.pose_publisher = self.create_publisher(Pose, "visual_infer/pose", 10)

        if self.vis_infer:
            self.image_publisher = self.create_publisher(Image, "visual_infer/image_raw", 10)

        self.create_subscription(Image, self.params['/image_topic'], self._image_callback, 10)
        self.create_subscription(Altitude, self.params['/mavros_altitude_topic'], self._altitude_callback, 10)

        self.get_logger().info(f'Node visual_infer_node started!')

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
            self.get_logger().info('Altitude is not estimated')
            return

        image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        results = self.pipeline.process(image, altitude=self.altitude)

        if results['pose'] is not None:
            self.pose_publisher.publish(self.generate_pose_msg(results['pose']))
        else:
            self.get_logger().info(f'Error: {results["error"]}')

        if self.vis_infer and results['image'] is not None:
            self.image_publisher.publish(self.generate_img_msg(results['image']))

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
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = visual.shape[0]
        msg.width = visual.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * visual.shape[1]
        msg.data = np.array(visual).tobytes()

        return msg


def main():
    rclpy.init()

    vi = VisualInference()
    rclpy.spin(vi)

    vi.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
