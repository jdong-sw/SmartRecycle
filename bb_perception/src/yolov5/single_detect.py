import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from utils.torch_utils import select_device, load_classifier, time_synchronized
from utils.general import check_img_size, non_max_suppression, apply_classifier, scale_coords, xyxy2xywh, \
    strip_optimizer, set_logging, increment_path
from utils.datasets import letterbox
import random
import numpy as np
import torch

conf_thres = 0.5
iou_thres = 0.5

device = select_device('')
half = device.type != 'cpu'  # half precision only supported on CUDA

classes = [
    'Cardboard',
    'Chips Bag',
    'Disposable Cup',
    'Napkin',
    'Plastic Bottle',
    'Plastic Container',
    'Soda Can'
]

def detect(model, img0):

    imgsz = 224
    imgsz = check_img_size(imgsz, s=model.stride.max())
    if half:
        model.half()

    # Run inference
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
    
    # Pre-process image
    img = letterbox(img0, new_shape=imgsz)[0]

    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    pred = model(img, augment=False)[0]

    # Apply NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres)

    # Process detections
    for i, det in enumerate(pred):  # detections per image
        gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

    results = []
    pred = pred[0]
    for det in pred:
        bounds = det[:4].numpy()
        confidence = det[4].numpy()
        label = classes[int(det[5])]
        results.append((label, confidence, bounds))

    # Return results
    return results