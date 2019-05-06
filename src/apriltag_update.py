import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad


lr = tf.TransformListener()
br = tf.TransformBroadcaster()


def apriltag_callback(data):
# use apriltag pose detection to find where is the robot
    for detection in data.detections:
        id=detection.id
        if ((id == 1)or(id==0)or(id==2)or(id==3)):   # tag id is one the correct ones that are on game board
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
            pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base_est', parent_frame_id = '/map')


if __name__== '__main__':
    rospy.init_node("apriltag_localizer")
    apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    rospy.Spin()
