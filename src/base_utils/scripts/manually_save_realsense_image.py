#! /usr/bin/env python
import os
import cv2

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from cv_bridge import CvBridge, CvBridgeError


class ManualSaveRGBD(object):
    def __init__(
        self,
        mode,
        save_dir=None,
        rgb_topic_name=None,
        depth_topic_name=None,
        start_index=None,
    ) -> None:
        self.mode = mode
        self.rgb_topic_name = rgb_topic_name
        self.depth_topic_name = depth_topic_name
        self.save_dir = save_dir if save_dir is not None else os.path.dirname(__file__)
        self.index = start_index if start_index is not None else 1
        self.bridge = CvBridge()

        rospy.loginfo("Waiting camera_info")
        camera_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
        self.fx, self.fy, self.cx, self.cy = (
            camera_info.K[0],
            camera_info.K[4],
            camera_info.K[2],
            camera_info.K[5],
        )
        self.image_width, self.image_height = camera_info.width, camera_info.height
        rospy.loginfo("Get camera_info")

        mode = mode.lower()
        if mode == "rgb":
            rospy.loginfo("Mode: rgb")
            assert rgb_topic_name is not None
            self.rgb_subscriber = rospy.Subscriber(
                rgb_topic_name, Image, self.rgb_cb, queue_size=1
            )
            pass
        elif mode == "rgbd":
            rospy.loginfo("Mode: rgbd")
            assert rgb_topic_name is not None
            assert depth_topic_name is not None
            self.rgb_subscriber = message_filters.Subscriber(rgb_topic_name, Image)
            self.depth_subscriber = message_filters.Subscriber(depth_topic_name, Image)
            self.rgbd_subscriber = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_subscriber, self.depth_subscriber],
                1,
                0.5,
            )
            self.rgbd_subscriber.registerCallback(self.rgbd_cb)
            pass
        else:
            raise BaseException("Error mode")
        pass

    def rgbd_cb(self, rgb_msg, depth_msg):
        rospy.loginfo("Get RGBD Image!")
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        c_x = self.image_width // 2
        c_y = self.image_height // 2
        real_z = depth_image[c_y, c_x] * 0.001
        real_x = (c_x - self.cx) / self.fx * real_z
        real_y = (c_y - self.cy) / self.fy * real_z
        rospy.loginfo(
            "potion:x=%f,y=%f,z=%f", real_x, real_y, real_z
        )  # 输出图像中心点在相机坐标系下的x,y,z

        rgb_show_image = rgb_image.copy()
        cv2.circle(
            rgb_show_image, (c_x, c_y), color=(255, 0, 0), radius=3, thickness=-1
        )
        cv2.imshow("color_image", rgb_show_image)
        cv2.imshow("depth_image", depth_image)
        key = cv2.waitKey(500)
        if key == ord("s"):
            cv2.imwrite(
                os.path.join(self.save_dir, "{}_color.png".format(self.index)),
                rgb_image,
            )
            cv2.imwrite(
                os.path.join(self.save_dir, "{}_depth.png".format(self.index)),
                depth_image,
            )
            self.index += 1
            pass

        pass

    def rgb_cb(self, rgb_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        cv2.imshow("color_image", rgb_image)
        key = cv2.waitKey(1000)
        if key == ord("s"):
            cv2.imwrite(
                os.path.join(self.save_dir, "{}_color.png".format(self.index)),
                rgb_image,
            )
            self.index += 1
            pass
        pass


if __name__ == "__main__":
    rospy.init_node("manually_save_rgbd", anonymous=True)

    manually_saver = ManualSaveRGBD(
        mode="rgbd",
        rgb_topic_name="/camera/color/image_raw",
        depth_topic_name="/camera/depth/image_rect_raw",
        save_dir="/home/red0orange/data/CenterSnap_test_data",
        start_index=4,
    )

    rospy.spin()
