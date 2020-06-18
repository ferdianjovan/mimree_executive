#!/usr/bin/env python

import os
import cv2
import subprocess

import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class PlanPublisher(object):
    """
    Converting plan_graph to an image and publishing it at
    /rosplan_plan_dispatcher/plan_image
    """
    def __init__(self):
        self.cv_img = None
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/rosplan_plan_dispatcher/plan_image",
                                         Image,
                                         queue_size=10)
        rospy.Subscriber("/rosplan_plan_dispatcher/plan_graph", String,
                         self.plan_cb)
        rospy.Timer(rospy.Duration(0.1), self.plan_img_publish)

    def plan_img_publish(self, event):
        try:
            if self.cv_img is not None:
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(self.cv_img, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def plan_cb(self, msg):
        pkg_path = roslib.packages.get_pkg_dir('mimree_executive')
        dotpath = os.path.join(pkg_path, "pddl", "plan.dot")
        fopen = open(dotpath, 'w')
        fopen.write(msg.data)
        fopen.close()
        fjpg = open(os.path.join(pkg_path, "pddl", "plan.jpg"), 'w')
        subprocess.call(["dot", "-Tjpg", dotpath], stdout=fjpg)
        fjpg.close()
        try:
            self.cv_img = cv2.imread(
                os.path.join(pkg_path, "pddl", "plan.jpg"), cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node("mission_plan_publisher")
    PlanPublisher()
    rospy.spin()
