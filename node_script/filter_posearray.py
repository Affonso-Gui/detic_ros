#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseArray
from jsk_recognition_msgs.msg import ClusterPointIndices

# Filter empty poses and apply ROI
class FilterPoses:
    def __init__(self, input_topic, output_topic, input_indices_topic, output_indices_topic):
        self.sub = rospy.Subscriber(input_topic, PoseArray, self.republish)
        self.pub = rospy.Publisher(output_topic, PoseArray, queue_size=1)
        self.indices_sub = rospy.Subscriber(input_indices_topic, ClusterPointIndices, self.update)
        self.indices_pub = rospy.Publisher(output_indices_topic, ClusterPointIndices, queue_size=1)
        self.indices = None

    def update(self, msg):
        self.indices = msg

    def republish(self, msg):
        poses = [x for x in msg.poses if self.check_pose(x)]
        if poses:
            poses.sort(key=lambda p: p.position.y)
            idx = msg.poses.index(poses[0])
            msg.poses = [poses[0]]
            self.pub.publish(msg)
            self.republish_indices(idx, msg.header)

    def republish_indices(self, idx, header=None):
        try:
            if self.indices:
                self.indices.cluster_indices = [self.indices.cluster_indices[idx]]
                if header:
                    self.indices.header=header
                self.indices_pub.publish(self.indices)
        except IndexError:
            pass

    def check_pose(self, pose):
        return (-0.15 < pose.position.z < -0.05)


if __name__ == '__main__':
    rospy.init_node('filter_pose_array', anonymous=True)
    argv = rospy.myargv(argv=sys.argv)
    FilterPoses(*argv[1:])
    rospy.spin()
