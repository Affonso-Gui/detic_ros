#!/usr/bin/env python3
import argparse
import os
from typing import List, Optional
import pickle

import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
import tqdm
from moviepy.editor import ImageSequenceClip

from node_config import NodeConfig
from wrapper import DeticWrapper


def bag_to_images(file_path: str, topic_name_extract: Optional[str] = None):
    bag = rosbag.Bag(file_path)
    image_list_: List[Image] = []

    for topic_name, msg, t in bag.read_messages():

        if topic_name_extract is None:
            if msg.__class__.__name__ == '_sensor_msgs__Image':
                image_list_.append(msg)
        else:
            if topic_name == topic_name_extract:
                image_list_.append(msg)

    def deep_cast(msg):
        # must exist etter way ... but I don't have time
        # see:
        # https://github.com/ros/ros_comm/issues/769
        msg_out = Image()
        msg_out.header.seq = msg.header.seq
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = msg.header.frame_id
        msg_out.height = msg.height
        msg_out.width = msg.width
        msg_out.encoding = msg.encoding
        msg_out.is_bigendian = msg.is_bigendian
        msg_out.step = msg.step
        msg_out.data = msg.data
        return msg_out

    image_list = [deep_cast(msg_) for msg_ in image_list_]
    return image_list


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=str, help='input file path')
    parser.add_argument('-topic', type=str, default='', help='topic name')
    parser.add_argument('-n', type=int, default=-1, help='number of image to be processed')
    parser.add_argument('-out', type=str, default='', help='out file path')
    parser.add_argument('-th', type=float, default=0.5, help='confidence threshold')
    parser.add_argument('-device', type=str, default='auto', help='device name')

    args = parser.parse_args()
    input_file_path = args.input
    topic_name = args.topic
    output_file_name = args.out
    confidence_threshold = args.th
    device = args.device
    n = args.n

    assert device in ['cpu', 'cuda', 'auto']

    rw, ext_input = os.path.splitext(input_file_path)
    assert ext_input == '.bag'

    if output_file_name == '':
        output_file_name = rw + '_segmented.pkl'

    debug_file_name = rw + '_debug.gif'

    if topic_name == '':
        topic_name = None

    image_list = bag_to_images(input_file_path, topic_name)

    if n != -1:
        image_list = image_list[:n]

    assert len(image_list) > 0
    print('{} images found'.format(len(image_list)))

    node_config = NodeConfig.from_args(True, False, False, confidence_threshold, device)
    detic_wrapper = DeticWrapper(node_config)

    result_dict = {'image': [], 'seginfo': [], 'debug_image': []}
    for image in tqdm.tqdm(image_list):
        seginfo, debug_image, _ = detic_wrapper.infer(image)
        result_dict['image'].append(image)
        result_dict['seginfo'].append(seginfo)
        result_dict['debug_image'].append(debug_image)


    with open(output_file_name, 'wb') as f:
        pickle.dump(result_dict, f)

    # dump debug gif image
    bridge = CvBridge()
    convert = lambda msg: bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    images = list(map(convert, result_dict['debug_image']))
    clip = ImageSequenceClip(images, fps=20)
    clip.write_gif(debug_file_name, fps=20)
