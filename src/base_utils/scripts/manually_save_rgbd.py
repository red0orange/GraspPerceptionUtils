#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError


def callback(data1, data2):
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(data1, "bgr8")
    depth_image = bridge.imgmsg_to_cv2(data2, "16UC1")
    cv2.imshow("color_image", color_image)
    cv2.waitKey(1000)
    c_x = 320
    c_y = 240
    real_z = depth_image[c_y, c_x] * 0.001
    real_x = (c_x - ppx) / fx * real_z
    real_y = (c_y - ppy) / fy * real_z
    rospy.loginfo(
        "potion:x=%f,y=%f,z=%f", real_x, real_y, real_z
    )  # 输出图像中心点在相机坐标系下的x,y,z


if __name__ == "__main__":
    global fx, fy, ppx, ppy  # 相机内参
    fx = 609.134765
    fy = 608.647949
    ppx = 312.763214
    ppy = 240.882049

    rospy.init_node("manually_save_rgbd", anonymous=True)

    rospy.loginfo("Waiting camera_info")
    camera_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
    rospy.loginfo("Get camera_info")

    color = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth = message_filters.Subscriber(
        "/camera/aligned_depth_to_color/image_raw", Image
    )
    color_depth = message_filters.ApproximateTimeSynchronizer(
        [color, depth], 10, 1, allow_headerless=True
    )
    color_depth.registerCallback(callback)
    rospy.spin()
