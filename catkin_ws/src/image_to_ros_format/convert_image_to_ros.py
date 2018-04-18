#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    # Publish to the MarkerLocator's original topic
    self.image_pub = rospy.Publisher("/markerlocator/image_raw",Image)

    # Subscribe from the Iris camera topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/iris/camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Display the image
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_gray, "8UC1"))
    except CvBridgeError as e:
      print(e)

def main(args):

  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
