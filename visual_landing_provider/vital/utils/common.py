from pathlib import Path
import yaml


def parse_yaml_params(path):
    """Parse ROS parameters.

    Returns:
        dict: Parsed ROS parameters.
    """
    with open(Path(path, 'config.yaml'), 'r') as f:
        try:
            params = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)

    params['/vis_infer'] = params['vis_infer']
    params['/image_topic'] = params['image_topic']
    params['/mavros_altitude_topic'] = params['mavros_altitude_topic']
    
    params['/camera/resolution'] = params['camera']['resolution']
    params['/camera/angles'] = params['camera']['angles']

    params['/density_estimator/path'] = params['density_estimator']['path']
    params['/density_estimator/onnx_provider'] = params['density_estimator']['onnx_provider']
    params['/density_estimator/conf_thresh'] = params['density_estimator']['conf_thresh']

    params['/keypoints_detector/path'] = params['keypoints_detector']['path']
    params['/keypoints_detector/onnx_provider'] = params['keypoints_detector']['onnx_provider']
    params['/keypoints_detector/conf_thresh'] = params['keypoints_detector']['conf_thresh']
    params['/keypoints_detector/std_thresh'] = params['keypoints_detector']['std_thresh']

    params['/rotation_estimator/camera_matrix'] = params['rotation_estimator']['camera_matrix']
    params['/rotation_estimator/dist_coeffs'] = params['rotation_estimator']['dist_coeffs']
    params['/rotation_estimator/target_real_model'] = params['rotation_estimator']['target_real_model']


    return params
