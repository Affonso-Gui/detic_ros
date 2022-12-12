#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseArray

class FilterEmpty:
    def __init__(self, input_topic, output_topic):
        self.sub = rospy.Subscriber(input_topic, PoseArray, self.republish)
        self.pub = rospy.Publisher(output_topic, PoseArray, queue_size=1)

    def republish(self, msg):
        if msg.poses:
            self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('filter_pose_array', anonymous=True)
    argv = rospy.myargv(argv=sys.argv)
    FilterEmpty(*argv[1:])
    rospy.spin()
