import cv2
import numpy as np
import onnxruntime as ort
import os
from typing import Tuple


class KeypointsDetector:
    def __init__(self, base_path: str, model_paths: str, provider: str, conf_thresh: float, std_thresh: float):
        self.conf_thresh = float(conf_thresh)
        self.std_thresh = float(std_thresh)

        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        providers = {
            'cpu': ('CPUExecutionProvider', {}),
            'cuda': ('CUDAExecutionProvider', {'device_id': 0}),
            'tensorrt': ('TensorrtExecutionProvider', {'device_id': 0, 'trt_fp16_enable': True, 'trt_engine_cache_enable': True}),
        }

        self.session = ort.InferenceSession(
            os.path.join(base_path, model_paths), options, providers=[providers[provider]])

        inputs = self.session.get_inputs()

        self.input_shape = inputs[0].shape
        self.input_name = inputs[0].name

        # WARMUP
        self.session.run(None, {self.input_name: np.random.rand(*self.input_shape).astype(np.float32)})

    def preprocess(self, img):
        inputs = cv2.resize(img, (self.input_shape[3], self.input_shape[2]))
        inputs = np.transpose(inputs, (2, 0, 1))
        inputs = inputs.astype(np.float32)
        inputs = np.expand_dims(inputs, 0)
        inputs = inputs*1./255.
        return inputs

    def predict(self, inputs):
        inputs = self.preprocess(inputs)

        outs = self.session.run(None, {self.input_name: inputs})

        return self.postprocess(outs)

    def postprocess(self, outputs):
        pad_conf = outputs[0][0]
        points_std = outputs[1][0]
        points = outputs[2][0]

        points = points.reshape(-1, 2)

        return points, pad_conf, points_std
