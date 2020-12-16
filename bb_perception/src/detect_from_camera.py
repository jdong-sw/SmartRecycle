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
from std_msgs.msg import String, ColorRGBA
import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2
import numpy as np
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3
import ros_numpy
import struct


class YoloDetector():
    def __init__(self):
        rospy.loginfo('Starting yolo video node...')
        rospy.init_node('yolo', anonymous=True)
        self.bridge = CvBridge()
        self.pointcloud = None
        self.saved = False

        rospy.loginfo('Loading model...')
        device = select_device('')
        half = device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        imgsz = 224
        self.model = attempt_load(f'{os.path.dirname(__file__)}/yolov5/runs/train/exp4/weights/best.pt', map_location=device)
        imgsz = check_img_size(imgsz, s=self.model.stride.max())
        if half:
            self.model.half()

        # Can you recycle dict
        class_dict = dict()
        class_dict['Cardboard'] = True
        class_dict['Chips Bag'] = False
        class_dict['Disposable Cup'] = False
        class_dict['Napkin'] = False
        class_dict['Plastic Bottle'] = True
        class_dict['Plastic Container']=True
        class_dict['Soda Can']=True        
        self.class_dict=class_dict

        # Subscribe to pointcloud to make predictions
        rospy.loginfo('Subscribing to camera pointcloud topic...')
        rospy.Subscriber('camera/depth_registered/points',
                         PointCloud2, self.process_pointcloud)

        rospy.loginfo('Awaiting images...')

        # Create publisher to publish predictions
        self.predict_pub=rospy.Publisher(
            'yolo/detections', String, queue_size=10)

        # Create publisher to publish markers to RViz
        self.rviz_pub=rospy.Publisher(
            'visualization_marker', Marker, queue_size=10)

    def process_pointcloud(self, msg):
        # Convert sensor_msgs PointCloud2 message to numpy array
        pointcloud=ros_numpy.numpify(msg)

        # Extract rgb image
        rgb=pointcloud['rgb']
        width=msg.width
        height=msg.height
        image=np.zeros((height, width, 3))

        for r in range(height):
            for c in range(width):
                image[r, c]=np.array(list(struct.pack('f', rgb[r, c])))[
                    [0, 1, 2]]

        # Run model on image
        # if (not self.saved):
        #     self.saved = True
        #     cv2.imwrite('test_im.jpg', image)
        preds=detect(self.model, image)

        # Publish results
        if (len(preds) > 0):
            self.predict_pub.publish(str(preds))

            # Check if we have pointcloud info
            for i, pred in enumerate(preds):
                # Get prediction
                label, confidence, bounds=pred
                recyclable=self.class_dict[label]
                x1, y1, x2, y2=bounds
                print(f'Detected {label} at ({x1},{y1},{x2},{y2})')

                # Make box slightly smaller
                bounds_width=0.8*(x2 - x1)
                bounds_height=0.8*(y2 - y1)
                bounds_x1=0.1*(x2 - x1) + x1
                bounds_x2=bounds_x1 + bounds_width
                bounds_y1=0.1*(y2 - y1) + y1
                bounds_y2=bounds_y1 + bounds_height

                # Get 3D ROI
                min_x=np.nanmin(pointcloud['x'][int(bounds_y1):int(bounds_y2), int(bounds_x1): int(bounds_x2)])
                max_x=np.nanmax(pointcloud['x'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])
                min_y=np.nanmin(pointcloud['y'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])
                max_y=np.nanmax(pointcloud['y'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])
                min_z=np.nanmin(pointcloud['z'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])
                max_z=np.nanmax(pointcloud['z'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])

                x=(min_x + max_x)/2
                y=(min_y + max_y)/2
                z=(min_z + max_z)/2
                width=max_x - min_x
                height=max_y - min_y
                depth=max_z - min_z
                print(f'{label} at ({x},{y},{z}) of size ({width},{height},{depth})')

                center_x=np.nanmean(pointcloud['z'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])
                center_y=-np.nanmean(pointcloud['x'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])
                center_z=-np.nanmean(pointcloud['y'][int(bounds_y1): int(bounds_y2), int(bounds_x1): int(bounds_x2)])

                # Generate markers for rviz
                self.pub_3D_marker(i, recyclable, center_x,
                                   center_y, center_z, 0.1, 0.1, 0.1)



                # Publish Text marker for label
                self.pub_text_marker(i, label, recyclable, center_x, center_y, center_z)

            else:
                return

    def pub_text_marker(self, id, label, r, x, y, z):
        marker=Marker()
        marker.header.frame_id='camera_aligned_depth_to_color_frame'
        marker.ns='label'
        marker.text = label
        marker.id=id
        marker.type=9  # text
        marker.action=0
        marker.lifetime=rospy.Duration(1)

        # Pose
        pose=Pose()
        pose.position.x=x
        pose.position.y=y
        pose.position.z=z + 0.05
        pose.orientation.w=1
        marker.pose=pose

        # Scale
        scale=Vector3()
        scale.z=0.035
        marker.scale=scale

        # Color
        color=ColorRGBA()
        color.a=1
        if r:
            color.r=0
            color.g=255
            color.b=0
        else:
            color.r=255
            color.g=0
            color.b=0

        marker.color=color

        # Publish marker
        self.rviz_pub.publish(marker)

    def pub_3D_marker(self, id, r, x, y, z, w, h, d):
        marker=Marker()
        marker.header.frame_id='camera_aligned_depth_to_color_frame'
        marker.ns='object'
        marker.id=id
        marker.type=1  # cube
        marker.action=0
        marker.lifetime=rospy.Duration(1)

        # Pose
        pose=Pose()
        pose.position.x=x
        pose.position.y=y
        pose.position.z=z
        pose.orientation.w=1
        marker.pose=pose

        # Scale
        scale=Vector3()
        scale.x=w
        scale.y=h
        scale.z=d
        marker.scale=scale

        # Color
        color=ColorRGBA()
        color.a=0.25
        if r:
            color.r=0
            color.g=255
            color.b=0
        else:
            color.r=255
            color.g=0
            color.b=0

        marker.color=color

        # Publish marker
        self.rviz_pub.publish(marker)


if __name__ == '__main__':
    try:
        YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
