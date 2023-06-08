#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage


TOPIC_NAME = '/r2d2/camera_node/image/compressed'
class EdgeDetectionNode(DTROS):
    def __init__(self, node_name):
        
        super(EdgeDetectionNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        # for converting from CompressedImage to cv2 and vice versa
        self.bridge = CvBridge()

        # subscribing to topic TOPIC_NAME, messaging object type is CompressedImage, on each notify callback is called
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1, buff_size="10MB")

        # publishing to the new topic 'image_pub', messaging object type is CompressedImage
        self.pub = rospy.Publisher('image_pub', CompressedImage, queue_size=1)

    def callback(self, msg):
        print(f'callback with type ${type(msg)}')

        # converting CompressedImage to cv2
        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # # applying filter
        # img_filtered = cv2.Canny(img_cv2, 50, 150)

        # # converting filtered result to CompressedImage
        # img_filtered_compressed = self.bridge.cv2_to_compressed_imgmsg(img_filtered)

        img_hsv = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2HSV)

# defining lower and upper bounds for yellow color in HSV
        # lower_yellow = np.array([20, 100, 100])
        # upper_yellow = np.array([40, 255, 255])

        # # creating a mask for yellow pixels
        # mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        # # applying the mask to the original image
        # img_yellow = cv2.bitwise_and(img_cv2, img_cv2, mask=mask)

        # # converting the masked image to grayscale
        # img_gray = cv2.cvtColor(img_yellow, cv2.COLOR_BGR2GRAY)

        # # applying Canny edge detection
        # img_edges = cv2.Canny(img_gray, 50, 150)

        # # converting the edge image to CompressedImage
        # img_edges_compressed = self.bridge.cv2_to_compressed_imgmsg(img_edges)
                

        hsv_img = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Create a mask for yellow pixels
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

        # Apply the mask to the original image
        yellow_filtered = cv2.bitwise_and(img_cv2, img_cv2, mask=yellow_mask)

        # Define lower and upper bounds for red color in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Create a mask for red pixels
        red_mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Apply the mask to the original image
        red_filtered = cv2.bitwise_and(img_cv2, img_cv2, mask=red_mask)

        # Define lower and upper bounds for white color in BGR
        lower_white = np.array([200, 200, 200])
        upper_white = np.array([255, 255, 255])

        # Create a mask for white pixels
        white_mask = cv2.inRange(img_cv2, lower_white, upper_white)

        # Apply the mask to the original image
        white_filtered = cv2.bitwise_and(img_cv2, img_cv2, mask=white_mask)

        # Combine the filtered images for yellow, red, and white lines
        lines_filtered = cv2.bitwise_or(yellow_filtered, red_filtered)
        lines_filtered = cv2.bitwise_or(lines_filtered, white_filtered)

        # Applying Canny edge detection on the filtered image
        img_filtered = cv2.Canny(lines_filtered, 50, 150)

        # Converting filtered result to CompressedImage
        img_filtered_compressed = self.bridge.cv2_to_compressed_imgmsg(img_filtered)


        # publishing to 'image_pub'
        self.pub.publish(img_filtered_compressed)


if __name__ == '__main__':
    node = EdgeDetectionNode(node_name='edge_detection_node')
    rospy.spin()
