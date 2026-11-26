"""YOLO inference via onnxruntime, without Ultralytics/PyTorch.

Supports the two ONNX layouts Ultralytics exports:
  * classic   [1, 4 + nc (+ nk), N]   (cx, cy, w, h, class scores, keypoints)  -> NMS here
  * end2end   [1, N, 6 (+ nk)]        (x1, y1, x2, y2, score, cls, keypoints)  -> no NMS needed

Detection models (bone_recognition_640.onnx) and pose models (yolo26n-pose.onnx,
17 keypoints as (x, y, conf)) share the same code path.

Export on the PC (Ultralytics only needed there):
    YOLO("model.pt").export(format="onnx", imgsz=640, simplify=True, opset=17)
"""
from dataclasses import dataclass, field
import ast

import cv2
import numpy as np
import onnxruntime as ort


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    cls: int
    keypoints: np.ndarray = field(default=None)   # (K, 3) = x, y, conf in image pixels

    @property
    def area(self):
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


class YoloOnnx:

    def __init__(self, model_path, num_threads=2, kpt_dims=3):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = int(num_threads)
        opts.inter_op_num_threads = 1
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.input_h, self.input_w = int(inp.shape[2]), int(inp.shape[3])

        meta = self.session.get_modelmeta().custom_metadata_map
        self.names = self._parse(meta.get('names'), {})
        kpt_shape = self._parse(meta.get('kpt_shape'), None)
        self.n_kpt = int(kpt_shape[0]) if kpt_shape else 0
        self.kpt_dims = int(kpt_shape[1]) if kpt_shape else kpt_dims
        self.end2end = str(meta.get('end2end', 'False')).lower() == 'true'

        out_shape = self.session.get_outputs()[0].shape
        nk = self.n_kpt * self.kpt_dims
        if not self.end2end and isinstance(out_shape[-1], int) and isinstance(out_shape[1], int):
            # classic layout has more anchors than channels; end2end has (N=300, 6+nk)
            self.end2end = out_shape[2] == 6 + nk and out_shape[1] > out_shape[2]
        self.nc = len(self.names) if self.names else (
            (out_shape[1] - 4 - nk) if not self.end2end else 1)

    @staticmethod
    def _parse(text, default):
        if not text:
            return default
        try:
            return ast.literal_eval(text)
        except (ValueError, SyntaxError):
            return default

    def _letterbox(self, frame):
        h, w = frame.shape[:2]
        scale = min(self.input_w / w, self.input_h / h)
        new_w, new_h = round(w * scale), round(h * scale)
        pad_x, pad_y = (self.input_w - new_w) / 2, (self.input_h - new_h) / 2
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        top, left = round(pad_y - 0.1), round(pad_x - 0.1)
        canvas = cv2.copyMakeBorder(
            resized, top, self.input_h - new_h - top, left, self.input_w - new_w - left,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return canvas, scale, left, top

    def __call__(self, frame_bgr, conf_thres=0.5, iou_thres=0.45):
        """Run the model on a BGR frame. Returns detections sorted by score (descending)."""
        img, scale, pad_x, pad_y = self._letterbox(frame_bgr)
        blob = np.ascontiguousarray(
            img[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        out = self.session.run(None, {self.input_name: blob})[0][0]
        nk = self.n_kpt * self.kpt_dims

        if self.end2end:                       # (N, 6 + nk)
            preds = out
            confs, cls_ids = preds[:, 4], preds[:, 5].astype(int)
            mask = confs >= conf_thres
            preds, confs, cls_ids = preds[mask], confs[mask], cls_ids[mask]
            boxes = preds[:, :4].copy()        # x1 y1 x2 y2
            kpts = preds[:, 6:6 + nk]
            keep = np.arange(len(preds))
        else:                                  # (4 + nc + nk, N) -> (N, ...)
            preds = out.T
            nc = preds.shape[1] - 4 - nk
            scores = preds[:, 4:4 + nc]
            cls_ids = scores.argmax(axis=1)
            confs = scores[np.arange(len(cls_ids)), cls_ids]
            mask = confs >= conf_thres
            preds, confs, cls_ids = preds[mask], confs[mask], cls_ids[mask]
            cx, cy, bw, bh = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
            boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1)
            kpts = preds[:, 4 + nc:4 + nc + nk]
            keep = np.arange(len(preds))
            if len(preds):
                # class-aware NMS: shift boxes per class so classes never suppress each other
                xywh = np.stack([boxes[:, 0], boxes[:, 1], bw, bh], axis=1)
                xywh[:, :2] += cls_ids[:, None] * 10000.0
                keep = np.array(cv2.dnn.NMSBoxes(
                    xywh.tolist(), confs.tolist(), conf_thres, iou_thres)).flatten()

        h, w = frame_bgr.shape[:2]
        dets = []
        for i in keep:
            x1, y1, x2, y2 = boxes[i]
            x1, x2 = (np.array([x1, x2]) - pad_x) / scale
            y1, y2 = (np.array([y1, y2]) - pad_y) / scale
            k = None
            if nk:
                k = kpts[i].reshape(self.n_kpt, self.kpt_dims).astype(np.float64).copy()
                k[:, 0] = (k[:, 0] - pad_x) / scale
                k[:, 1] = (k[:, 1] - pad_y) / scale
                if self.kpt_dims == 2:
                    k = np.hstack([k, np.ones((self.n_kpt, 1))])
            dets.append(Detection(
                float(np.clip(x1, 0, w)), float(np.clip(y1, 0, h)),
                float(np.clip(x2, 0, w)), float(np.clip(y2, 0, h)),
                float(confs[i]), int(cls_ids[i]), k))
        dets.sort(key=lambda d: d.score, reverse=True)
        return dets
