#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseArray

class FilterEmpty:
    def __init__(self, input_topic, output_topic):
        self.sub = rospy.Subscriber(input_topic, PoseArray, self.republish)
        self.pub = rospy.Publisher(output_topic, PoseArray, queue_size=1)

    def republish(self, msg):
        msg.poses = [x for x in msg.poses if self.check_pose(x)]
        if msg.poses:
            msg.poses.sort(key=lambda p: p.position.y)
            self.pub.publish(msg)

    def check_pose(self, pose):
        return (0.070 < pose.position.z < 0.73) and (pose.position.x < 0.50)


if __name__ == '__main__':
    rospy.init_node('filter_pose_array', anonymous=True)
    argv = rospy.myargv(argv=sys.argv)
    FilterEmpty(*argv[1:])
    rospy.spin()
