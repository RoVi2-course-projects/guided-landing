#!/usr/bin/env python2
# Global libraries
import signal
import sys
# Third party libraries
import cv2
from geometry_msgs.msg import Vector3
import rospy


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


class DeltaPosSubscriber(rospy.Subscriber):
    """
    Custom Subscriber that gets the position difference.
    """
    def __init__(self, sub_topic):
        rospy.Subscriber.__init__(self, sub_topic, Vector3, self.callback,
                                  queue_size=1)
        self.delta_pos = Vector3()

    def callback(self, position_delta):
        self.delta_pos = position_delta
        return


def set_position(pos_diff, uav_pos):
    """
    Get the position setpoint given the distance to the image centre.
    """



def main():
    sub_topic = "/position_diff"
    # Create a node and subscriber for getting the position differences.
    rospy.init_node("pos_controller", anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    delta_pos_subscriber = DeltaPosSubscriber(sub_topic)
    diff = Vector3()
    uav_pos = Vector3()
    # Main loop
    while not rospy.is_shutdown():
        pos_diff = delta_pos_subscriber.delta_pos
        set_position(pos_diff, uav_pos)
        rate.sleep()
    return


if __name__ == "__main__":
    main()
