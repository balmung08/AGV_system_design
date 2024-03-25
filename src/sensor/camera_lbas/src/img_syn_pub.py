#! /usr/bin/python2
# coding=utf-8


import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2


def toMsg(img):
    img_msg = bridge.cv2_to_imgmsg(img,"bgr8")
    return img_msg

def getImg(img_msg):
    img = bridge.imgmsg_to_cv2(img_msg,"bgr8")
    return img

def callback(img1_msg,img2_msg):
    img1 = getImg(img1_msg)
    img2 = getImg(img2_msg)
    syn_img = np.concatenate([img1,img2],axis=1)  # cat with width
    syn_img_msg = toMsg(syn_img)
    syn_pub.publish(syn_img_msg)
    # cv2.imshow("syn",cv2.resize(syn_img,(512,320)))
    # cv2.waitKey(1)

def main():
    rospy.init_node('img_syn_pub', anonymous=True)
    img_topic1 = rospy.get_param("~topic1","lbas_image1")
    img_topic2 = rospy.get_param("~topic2","lbas_image2")
    syn_tol = rospy.get_param("~syn_tol",0.05)
    img1_sub = message_filters.Subscriber(img_topic1, Image)
    img2_sub = message_filters.Subscriber(img_topic2, Image)
    syn = message_filters.ApproximateTimeSynchronizer([img1_sub,img2_sub], 5,syn_tol)
    syn.registerCallback(callback)
    print("init succedded!")
    rospy.spin()


if __name__ == "__main__":
    bridge = CvBridge()
    syn_pub = rospy.Publisher('syn_img',Image,queue_size=1)
    main()