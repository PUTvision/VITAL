import cv2
import numpy as np

from typing import Tuple


class RotationEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, real_model: np.ndarray):
        """ Rotation estimator calculates yaw angle from keypoints and real-world model using PnP algorithm.

        Parameters
        ----------
        camera_matrix : np.ndarray
            Camera matrix in shape (3,3)
        dist_coeffs : np.ndarray
            Distortion coefficients in shape (5,1)
        real_model : np.ndarray
            Real model keypoints in shape (n,3,1)
        """
        self.camera_matrix = np.array(camera_matrix).astype(np.double).reshape(3,3)
        self.dist_coeffs = np.array(dist_coeffs).astype(np.double).reshape(-1,1)
        self.real_model = np.array(real_model).astype(np.double).reshape(-1,3,1)

    def predict(self, keypoints_global: np.ndarray) -> Tuple[np.ndarray, float]:
        """ 

        Parameters
        ----------
        keypoints_global : np.ndarray
            Image model keypoints in global coordinates

        Returns
        -------
        np.ndarray
            Translation vector in shape (3,1)
        float
            Yaw angle in radians
        """
 
        ret, rvec, tvec = cv2.solvePnP(
            objectPoints = self.real_model,
            imagePoints = keypoints_global.reshape(-1,2,1).astype(np.double),
            cameraMatrix = self.camera_matrix, 
            distCoeffs = self.dist_coeffs,
            useExtrinsicGuess=False,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if ret:
            R = cv2.Rodrigues(rvec)[0]
            yaw = np.arctan2(R[1,0], R[0,0])

            return tvec, yaw
        else:
            return tvec, None
