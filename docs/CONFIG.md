# Config documentation

[Back to README](../README.md)


```
image_topic: /camera/rgb/image_raw
mavros_altitude_topic: /mavros/altitude
vis_infer: True
```

```
camera:
  resolution: [1920, 1080]
  angles: [69.0, 54.0]
```

```
density_estimator:
  path: './data/density_estimator.onnx'
  onnx_provider: cpu # tensorrt, cuda or cpu
  conf_thresh: 0.6
```

```
keypoints_detector:
  path: './data/keypoints_detector.onnx'
  onnx_provider: cpu # tensorrt, cuda or cpu
  conf_thresh: 0.6
  std_thresh: 1e-3
```

```
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
