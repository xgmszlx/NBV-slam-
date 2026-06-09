#!/usr/bin/env python3
"""YOLOv8 + BoT-SORT semantic tracking ROS node scaffold."""

import math

import rospy
from sensor_msgs.msg import Image

from hnbv_msgs.msg import TrackedObjectArray


class SemanticTrackerNode:
    def __init__(self):
        self.confidence = rospy.get_param("~confidence", 0.8)
        self.nms_iou = rospy.get_param("~nms_iou", 0.45)
        self.dynamic_class = rospy.get_param("~dynamic_class", "person")
        self.tracks_pub = rospy.Publisher("/hnbv/semantics/tracks", TrackedObjectArray, queue_size=1)
        self.mask_pub = rospy.Publisher("/hnbv/semantics/instance_mask", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.on_image, queue_size=1)

    def on_image(self, image_msg):
        tracks = TrackedObjectArray()
        tracks.header = image_msg.header
        self.tracks_pub.publish(tracks)

        mask = Image()
        mask.header = image_msg.header
        mask.height = image_msg.height
        mask.width = image_msg.width
        mask.encoding = "32SC1"
        mask.step = image_msg.width * 4
        mask.data = bytes(mask.step * mask.height)
        self.mask_pub.publish(mask)


def main():
    rospy.init_node("semantic_tracker_node")
    SemanticTrackerNode()
    rospy.loginfo("semantic_tracker_node scaffold started")
    rospy.spin()


if __name__ == "__main__":
    main()

