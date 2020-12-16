#!/home/johnisking/pyenvs/yolo/bin/python
import rospy
import cv2
from yolov5.single_detect import detect
from yolov5.utils.torch_utils import select_device, load_classifier, time_synchronized
from yolov5.models.experimental import attempt_load
from yolov5.utils.general import check_img_size, non_max_suppression, apply_classifier, scale_coords, xyxy2xywh, \
    strip_optimizer, set_logging, increment_path
import os
import sys


def process_sample(image_filepath):
    rospy.loginfo('Starting yolo sample node...')
    rospy.init_node('sample', anonymous=True)

    rospy.loginfo('Loading model...')
    device = select_device('')
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    imgsz = 224
    model = attempt_load(f'{os.path.dirname(__file__)}/yolov5/runs/train/exp4/weights/best.pt', map_location=device)
    imgsz = check_img_size(imgsz, s=model.stride.max())
    if half:
        model.half()
    
    rospy.loginfo('Reading image...')
    im = cv2.imread(image_filepath)
    if (im is None):
        rospy.loginfo('Failed to read image.')
        return

    rospy.loginfo('Running inference...')
    preds = detect(model, im)

    rospy.loginfo('Predictions:')
    for pred in preds:
        rospy.loginfo(f'    Class: {pred[0]}')
        rospy.loginfo(f'        Confidence: {pred[1]}')
        rospy.loginfo(f'        Bounds: {pred[2]}')


    

if __name__=='__main__':
    try:
        process_sample(r'/home/johnisking/test_im.jpg')
    except rospy.ROSInterruptException:
        pass