import cv2
import numpy as np
import onnxruntime as ort
import os
from typing import Tuple


class DensityEstimator:
    def __init__(self, base_path: str, model_paths: str, provider: str, conf_thresh: float):
        self.conf_thresh = conf_thresh
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
        inputs, ratio, dwdh = self.letterbox(img, auto=False, new_shape=(self.input_shape[2], self.input_shape[3]), color=(0,0,0))
        inputs = np.transpose(inputs, (2, 0, 1))
        inputs = inputs.astype(np.float32)
        inputs = np.expand_dims(inputs, 0)
        inputs = inputs*1./255.
        return inputs, ratio, dwdh

    def predict(self, img):
        inputs, ratio, dwdh = self.preprocess(img)

        outs = self.session.run(None, {self.input_name: inputs})

        return self.postprocess(outs, ratio, dwdh, img)

    def postprocess(self, outputs, ratio, dwdh, img):
        outs = outputs[0]
        people, pad = outs.squeeze()[1], outs.squeeze()[2]

        people = people[int(dwdh[1]):-max(int(dwdh[1]), 1), int(dwdh[0]):-max(1, int(dwdh[0]))]
        pad = pad[int(dwdh[1]):-max(int(dwdh[1]), 1), int(dwdh[0]):-max(1, int(dwdh[0]))]

        people = cv2.resize(people, (img.shape[1], img.shape[0]))
        pad = cv2.resize(pad, (img.shape[1], img.shape[0]))

        people[people < self.conf_thresh] = 0
        pad[pad < self.conf_thresh] = 0

        return people, pad


    @staticmethod
    def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleup=True, stride=32):
        # Resize and pad image while meeting stride-multiple constraints
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return im, r, (dw, dh)
