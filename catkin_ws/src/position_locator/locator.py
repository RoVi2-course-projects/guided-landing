#!/usr/bin/env python2
# Global libraries
import signal
import sys
# Third party libraries
import cv2
from geometry_msgs.msg import Vector3
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
# Local libraries
import img_processing


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


class ImgSubscriber(rospy.Subscriber):
    """
    Create a custom image Subscriber, overriding ROS native one.
    """
    def __init__(self, sub_topic):
        rospy.Subscriber.__init__(self, sub_topic, CompressedImage,
                                  self.img_callback, queue_size=1)
        self.pos_diff = [None, None]

    def img_callback(self, img_data):
        # Format the image to CV2, array shape.
        img_array = np.fromstring(img_data.data, np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        im_bw, square_cnt = img_processing.get_marker_position(img)
        # Get the centre of the square.
        if square_cnt is not None:
            centre_x, centre_y = img_processing.get_marker_centre(square_cnt)
            # Get distance to image centre
            diff_x = im_bw.shape[0]/2 - centre_x
            diff_y = im_bw.shape[1]/2 - centre_y
            self.pos_diff = [diff_x, diff_y]
            # Plot the binary image with the found marker.
            im_bw = cv2.cvtColor(im_bw, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(im_bw, [square_cnt], -1, (255, 0, 0), 3)
            cv2.circle(im_bw, (centre_x, centre_y), 7, (255, 255, 0), -1)
        cv2.imshow('cv_img', im_bw)
        cv2.waitKey(2)
        return


def main():
    sub_topic = "/iris/camera/image_raw/compressed"
    pub_topic = "/position_diff"
    # Create a node and subscriber for getting the images
    rospy.init_node("locator", anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    img_subscriber = ImgSubscriber(sub_topic)
    publisher = rospy.Publisher(pub_topic, Vector3, queue_size=1)
    location = Vector3()
    # Loop for getting images and running the processing algorithm
    while not rospy.is_shutdown():
        location.x = img_subscriber.pos_diff[0]
        location.y = img_subscriber.pos_diff[1]
        print("Diff is ({}, {})".format(location.x, location.y))
        publisher.publish(location)
        rate.sleep()
    return


if __name__ == '__main__':
    main()
