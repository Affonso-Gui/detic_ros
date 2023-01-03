#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseArray

class FilterEmpty:
    def __init__(self, input_topic, output_topic, output_topic_table):
        self.sub = rospy.Subscriber(input_topic, PoseArray, self.republish)
        self.pub = rospy.Publisher(output_topic, PoseArray, queue_size=1)
        self.pub_table = rospy.Publisher(output_topic_table, PoseArray, queue_size=1)

    def republish(self, msg):
        if msg.poses:
            self.pub.publish(msg)
            self.republish_table(msg)

    def republish_table(self, msg):
        msg.poses = [x for x in msg.poses if self.check_pose(x)]
        if msg.poses:
            self.pub_table.publish(msg)

    def check_pose(self, pose):
        return (0.070 < pose.position.z < 0.73)


if __name__ == '__main__':
    rospy.init_node('filter_pose_array', anonymous=True)
    argv = rospy.myargv(argv=sys.argv)
    FilterEmpty(*argv[1:])
    rospy.spin()
