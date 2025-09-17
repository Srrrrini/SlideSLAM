#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageMasker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/spot_image", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/masked_spot_image", Image, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Mask columns greater than 1200 as black
        height, width, _ = cv_image.shape
        if width > 1200:
            cv_image[:, 1200:] = 0

        try:
            masked_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        self.image_pub.publish(masked_image_msg)

def main():
    rospy.init_node('image_masker', anonymous=True)
    image_masker = ImageMasker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
