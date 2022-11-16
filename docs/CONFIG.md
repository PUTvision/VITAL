# Config documentation

[Back to README](../README.md)


## Global variables

  - `image_topic` - topic name for the input image
  - `mavros_altitude_topic` - topic name for the altitude from mavros
  - `vis_infer` - if true, visualization topics will be published

```yaml
image_topic: /camera/rgb/image_raw
mavros_altitude_topic: /mavros/altitude
vis_infer: True
```

## Camera parameters

- `resolution` - camera input resolution in pixels
- `angles` - camera angles in degrees (horizontal, vertical)

```yaml
camera:
  resolution: [1920, 1080]
  angles: [69.0, 54.0]
```

## Landing target detector model parameters

  - `path` - relative path to the model
  - `onnx_provider` - ONNX provider (tensorrt (jetson), cuda or cpu)
  - `conf_thresh` - confidence threshold for the detector

```yaml
density_estimator:
  path: './data/density_estimator.onnx'
  onnx_provider: cpu # tensorrt, cuda or cpu
  conf_thresh: 0.6
```

## Region of interest keypoionts model parameters

  - `path` - relative path to the model
  - `onnx_provider` - ONNX provider (tensorrt (jetson), cuda or cpu)
  - `conf_thresh` - confidence threshold for the detector
  - `std_tresh` - standard deviation threshold for the detector

```yaml
keypoints_detector:
  path: './data/keypoints_detector.onnx'
  onnx_provider: cpu # tensorrt, cuda or cpu
  conf_thresh: 0.6
  std_thresh: 1e-3
```

## Rotation estimation algorithm parameters

  - `camera_matrix` - camera matrix (3x3) from the camera calibration
  - `distortion_coefficients` - distortion coefficients (1x5) from the camera calibration
  - `target_real_model` - real model of the landing target (3x4) in meters

```yaml
rotation_estimator:
  camera_matrix: [
    [1.44273560e+03, 0.00000000e+00, 960],
    [0.00000000e+00, 1.44273560e+03, 540],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
  ]
  dist_coeffs: [
    [ 8.94706541e-02, -4.60232377e-02,  7.07396345e-03, -2.76138997e-04, -2.76970285e-01]
  ]
  target_real_model: [
    [0.0, 0.0, 0.0], #middle
    [0.0, -0.227, 0.0], #up
    [0.204, 0.116, 0.0], #right
    [-0.204, 0.116, 0.0], #left      
  ]
```

[Back to README](../README.md)
