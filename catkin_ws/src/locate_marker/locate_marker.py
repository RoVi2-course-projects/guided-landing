#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point

class LocateMarker:

  def __init__(self):
    # Publish to the MarkerLocator's original topic
    #self.image_pub = rospy.Publisher("/markerlocator/image_raw",Image)

    # publish point
    self.loc_pub = rospy.Publisher("location", Point, queue_size=10)

    # Subscribe from the Iris camera topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/markerlocator/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except CvBridgeError as e:
      print(e)

    # threshold
    ret,thresh1 = cv2.threshold(cv_image,200,255,cv2.THRESH_BINARY)
    im2,contours,hierarchy = cv2.findContours(thresh1, 1, 2)
    witdh, height = cv_image.shape[:2]
    print(witdh, height)

    if contours:
        cnt = contours[0]
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print(cx,cy)
        #print( M )
        cv2.circle(cv_image,(cx,cy), 20, (255,0,0), -1)
        point = Point(cy-height/2, cx-witdh/2, 0.0)

    else:
        point = Point(0,0,0)

    # Display the image
    #cv2.line(cv_image,(0,0),(511,511),(255,0,0),5)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    #frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

    try:
      print("none")
      self.loc_pub.publish(point)
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_gray, "8UC1"))
    except CvBridgeError as e:
      print(e)

def main(args):

  rospy.init_node('image_converter', anonymous=True)
  ic = LocateMarker()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
