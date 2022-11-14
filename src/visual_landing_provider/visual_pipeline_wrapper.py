import cv2
import numpy as np

from visual_landing_provider.nn.density_estimator import DensityEstimator
from visual_landing_provider.nn.keypoints_detector import KeypointsDetector
from visual_landing_provider.nn.rotation_estimator import RotationEstimator

class VisualPipelineWrapper:
    def __init__(self, base_path: str, config: dict, visualize: bool = False):

        self.width, self.height = [int(x) for x in config['/camera/resolution']]
        self.visualize = visualize
        
        self.density_estimator = DensityEstimator(
            base_path = base_path,
            model_paths = config['/density_estimator/path'],
            provider = config['/density_estimator/onnx_provider'],
            conf_thresh = config['/density_estimator/conf_thresh']
            )

        self.keypoints_detector = KeypointsDetector(
            base_path = base_path,
            model_paths = config['/keypoints_detector/path'],
            provider = config['/keypoints_detector/onnx_provider'],
            conf_thresh = config['/keypoints_detector/conf_thresh'],
            std_thresh = config['/keypoints_detector/std_thresh']
            )

        self.rotation_estimator = RotationEstimator(
            camera_matrix = config['/rotation_estimator/camera_matrix'], 
            dist_coeffs = config['/rotation_estimator/dist_coeffs'], 
            real_model = config['/rotation_estimator/target_real_model'],
        )

        self.camera_angles_tan = [
            np.tan(np.radians(config['/camera/angles'][0]/2)),
            np.tan(np.radians(config['/camera/angles'][1]/2))
        ]

        self.thresh = 0.6
        self.rotation_yaw = []
    
    def process(self, image: np.ndarray, altitude: float = None, depth: np.array = None) -> dict:
        """ Calculates Pose from image.

        Parameters
        ----------
        image : np.ndarray
            Input image in shape (height, width, 3)
        altitude : float, optional
            Altitude in meters, by default None
        depth : np.array, optional
            Depth image in shape (height, width), by default None

        Returns
        -------
        dict
            Results of the pipeline. Contains: error, pose, image.
        """

        assert altitude is not None or depth is not None

        results = {
            'error': 'no_error',
            'pose': None,
            'image': None
        }

        if self.visualize:
            results['image'] = image.copy()
        
        people, landing_pad = self.density_estimator.predict(image)

        people = cv2.dilate(people, np.ones((5,5)))
        landing_pad = cv2.dilate(landing_pad, np.ones((5,5)))

        people = cv2.medianBlur(people,5)
        landing_pad = cv2.medianBlur(landing_pad,5)

        thresholded_pad = (landing_pad > self.density_estimator.conf_thresh).astype(np.uint8)
        cnts = cv2.findContours(thresholded_pad, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        thresholded_people = (people > self.density_estimator.conf_thresh).astype(np.uint8)
        cnts_people = cv2.findContours(thresholded_people, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        if len(cnts) == 0:
            results['error'] = 'no landing pad detected, no cnts'
            return results

        biggest_blob_pad = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)[0]
        if cv2.contourArea(biggest_blob_pad) < 30.0:
            results['error'] = 'no landing pad detected, no big cnts'
            return results

        blobs_people = sorted(cnts_people, key=lambda x: cv2.contourArea(x), reverse=True)
        if len(blobs_people) > 0 and cv2.contourArea(blobs_people[0]) > 100*100:
            results['error'] = 'people detected'
            return results
        
        x, y, w, h = cv2.boundingRect(biggest_blob_pad)

        if self.visualize:
            cv2.rectangle(results['image'], (x, y), (x+w, y+h), (255, 0, 0), 2)

        pad_image = image[y:y+h, x:x+w].copy()

        keypoints, points_conf, points_std = self.keypoints_detector.predict(pad_image)

        c_x = (x + w/2)
        c_y = (y + h/2)

        if np.all(points_conf > self.keypoints_detector.conf_thresh) and np.all(points_std < self.keypoints_detector.std_thresh):
            keypoints_local = keypoints * np.array([w, h])
            keypoints_global = keypoints_local + np.array([x, y])

            if self.visualize:
                for (xl, yl) in keypoints_global:
                    cv2.circle(results['image'], (int(xl), int(yl)), 3, (0, 255, 0), -1)
            
            tvec, yaw = self.rotation_estimator.predict(keypoints_global)

            if tvec is not None and yaw is not None:
                if self.visualize:
                    cv2.putText(results['image'], f'Yaw: {np.degrees(yaw):.2f}', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                # flatten and swap axes to PX4 controller
                results['pose'] = np.array([
                    -tvec[1][0], tvec[0][0], tvec[2][0], 0.0, 0.0, yaw
                ])

                return results

        if self.visualize:
            cv2.circle(results['image'], (int(c_x), int(c_y)), 3 , (0, 255, 0), -1)

        lp_Y = c_x - self.width//2
        lp_X = self.height//2 - c_y

        lp_Y = lp_Y / (self.width//2)
        lp_X = lp_X / (self.height//2)

        if depth is not None:
            alt = depth[c_y, c_x]
        else:
            alt = altitude

        camera_X = alt * self.camera_angles_tan[0]
        camera_Y = alt * self.camera_angles_tan[1]

        lp_Y = lp_Y * camera_X
        lp_X = lp_X * camera_Y

        results['pose'] = np.array([
            lp_X, lp_Y, alt, 0, 0, 0
        ])

        return results
