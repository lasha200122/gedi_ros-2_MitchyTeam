#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage



class TestNode(DTROS):
    def __init__(self, node_name):
        super(TestNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        
        self.bridge = CvBridge()

        self.publisher = rospy.Publisher(
            'new_images',
            CompressedImage,
            queue_size=1
        )


    def get_image(self):
        img = np.zeros((200, 200, 3), np.uint8)
        center = (100, 100)
        radius = 50
        color = (0, 0, 255)
        thickness = 2

        cv2.circle(img, center, radius, color, thickness)
        return img
        

    def run(self):
        img = self.get_image() # image with type cv2
        img_compressed = self.bridge.cv2_to_compressed_imgmsg(img) # CompressedImage

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo('publishing messsage...')
            self.publisher.publish(img_compressed)
            rate.sleep()


if __name__ == '__main__':
    node = TestNode(node_name='test_node')
    node.run()
    rospy.spin()