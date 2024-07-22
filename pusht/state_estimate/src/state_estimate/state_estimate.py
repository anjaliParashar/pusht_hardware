#!/usr/bin/env python
import rospy
import cv2
from apriltag import apriltag
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os



class Camera_estimate:
    def __init__(self):
        rospy.init_node("state_estimatoR_NODE", anonymous=True)
        # Subscribe to depth images from ros param
        image_width = 16
        aspect = 4.0 / 3.0
        self.image_shape = (image_width, int(image_width / aspect))
        self.tag = None
        self.bridge = CvBridge()
        self.tag_sub = rospy.Subscriber(
            rospy.get_param("~depth_image_topic", "/camera/depth/image_rect_raw"),
            Image,
            self.depth_image_callback,
        )
        self.image_pub = rospy.Publisher("apriltag_img", Image, queue_size=1)

    def depth_image_callback(self, msg):
            original_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            #convert image to greyscale if it is color 
            img_gray = cv2.cvtColor(original_image, cv2.COLOR_RGB2GRAY)
            #self.depth_image = cv2.resize(
            #    original_image, self.image_shape, interpolation=cv2.INTER_AREA
            #)
            detector = apriltag("tagStandard41h12")
            detections = detector.detect(original_image)
            self.tag = detections

    def update(self):
        """
        Update and publish the state estimation.
        """
        if self.depth_image is not None:
            # Pack [x,y,theta,v] from state message into TimedPose2DObservation instance
            # Make sure to normalize the time
            print(self.tag)
        else:
            rospy.loginfo("No image available!")


if __name__ == "__main__":
    try:
        state_estimate_node = Camera_estimate()
        state_estimate_node.run()
    except rospy.ROSInterruptException:
        pass