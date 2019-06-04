#!/usr/bin/env python
# -*- coding:utf8 -*-

import rospy
import numpy as np
import cv2
from enum import Enum
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from turtlebot3_autorace_detect.cfg import DetectTrafficLightParamsConfig


class DetectTrafficLight():
    def __init__(self):
        self.hue_red_l = rospy.get_param("~detect/lane/red/hue_l", 0)
        self.hue_red_h = rospy.get_param("~detect/lane/red/hue_h", 10)
        self.saturation_red_l = rospy.get_param("~detect/lane/red/saturation_l", 30)
        self.saturation_red_h = rospy.get_param("~detect/lane/red/saturation_h", 255)
        self.lightness_red_l = rospy.get_param("~detect/lane/red/lightness_l", 48)
        self.lightness_red_h = rospy.get_param("~detect/lane/red/lightness_h", 255)

        self.hue_yellow_l = rospy.get_param("~detect/lane/yellow/hue_l", 20)
        self.hue_yellow_h = rospy.get_param("~detect/lane/yellow/hue_h", 35)
        self.saturation_yellow_l = rospy.get_param("~detect/lane/yellow/saturation_l", 100)
        self.saturation_yellow_h = rospy.get_param("~detect/lane/yellow/saturation_h", 255)
        self.lightness_yellow_l = rospy.get_param("~detect/lane/yellow/lightness_l", 50)
        self.lightness_yellow_h = rospy.get_param("~detect/lane/yellow/lightness_h", 255)

        self.hue_green_l = rospy.get_param("~detect/lane/green/hue_l", 46)
        self.hue_green_h = rospy.get_param("~detect/lane/green/hue_h", 76)
        self.saturation_green_l = rospy.get_param("~detect/lane/green/saturation_l", 86)
        self.saturation_green_h = rospy.get_param("~detect/lane/green/saturation_h", 255)
        self.lightness_green_l = rospy.get_param("~detect/lane/green/lightness_l", 50)
        self.lightness_green_h = rospy.get_param("~detect/lane/green/lightness_h", 255)

        self.is_calibration_mode = rospy.get_param("~is_detection_calibration_mode", False)

        if self.is_calibration_mode == True:
            srv_detect_lane = Server(DetectTrafficLightParamsConfig, self.cbGetDetectTrafficLightParam)

        self.sub_image_type = "raw"  # "compressed" / "raw"
        self.pub_image_type = "raw"  # "compressed" / "raw"

        self.counter = 1

        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage,
                                                       self.cbGetImage, queue_size=1)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/camera/image_compensated', Image, self.cbGetImage, queue_size=1)

        if self.pub_image_type == "compressed":
            # publishes compensated image in compressed type
            self.pub_image_traffic_light = rospy.Publisher('/detect/image_output/compressed', CompressedImage,
                                                           queue_size=1)
        elif self.pub_image_type == "raw":
            # publishes compensated image in raw type
            self.pub_image_traffic_light = rospy.Publisher('/detect/image_output', Image, queue_size=1)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes light image in compressed type
                self.pub_image_red_light = rospy.Publisher('/detect/image_output_sub1/compressed', CompressedImage,
                                                           queue_size=1)
                self.pub_image_yellow_light = rospy.Publisher('/detect/image_output_sub2/compressed', CompressedImage,
                                                              queue_size=1)
                self.pub_image_green_light = rospy.Publisher('/detect/image_output_sub3/compressed', CompressedImage,
                                                             queue_size=1)
            elif self.pub_image_type == "raw":
                # publishes light image in raw type
                self.pub_image_red_light = rospy.Publisher('/detect/image_output_sub1', Image, queue_size=1)
                self.pub_image_yellow_light = rospy.Publisher('/detect/image_output_sub2', Image, queue_size=1)
                self.pub_image_green_light = rospy.Publisher('/detect/image_output_sub3', Image, queue_size=1)

        self.sub_traffic_light_finished = rospy.Subscriber('/control/traffic_light_finished', UInt8,
                                                           self.cbTrafficLightFinished, queue_size=1)
        self.pub_traffic_light_return = rospy.Publisher('/detect/traffic_light_stamped', UInt8, queue_size=1)
        self.pub_parking_start = rospy.Publisher('/control/traffic_light_start', UInt8, queue_size=1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size=1)

        self.StepOfTrafficLight = Enum('StepOfTrafficLight',
                                       'searching_traffic_light searching_green_light searching_yellow_light searching_red_light waiting_green_light pass_traffic_light')

        self.cvBridge = CvBridge()
        self.cv_image = None

        self.is_image_available = False
        self.is_traffic_light_finished = False

        self.green_count = 0
        self.yellow_count = 0
        self.red_count = 0
        self.stop_count = 0
        self.off_traffic = False
        rospy.sleep(1)

        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_image_available == True:
                self.fnFindTrafficLight()

            loop_rate.sleep()

    def cbGetDetectTrafficLightParam(self, config, level):
        rospy.loginfo("[Detect Traffic Light] Detect Traffic Light Calibration Parameter reconfigured to")
        rospy.loginfo("hue_red_l : %d", config.hue_red_l)
        rospy.loginfo("hue_red_h : %d", config.hue_red_h)
        rospy.loginfo("saturation_red_l : %d", config.saturation_red_l)
        rospy.loginfo("saturation_red_h : %d", config.saturation_red_h)
        rospy.loginfo("lightness_red_l : %d", config.lightness_red_l)
        rospy.loginfo("lightness_red_h : %d", config.lightness_red_h)

        rospy.loginfo("hue_yellow_l : %d", config.hue_yellow_l)
        rospy.loginfo("hue_yellow_h : %d", config.hue_yellow_h)
        rospy.loginfo("saturation_yellow_l : %d", config.saturation_yellow_l)
        rospy.loginfo("saturation_yellow_h : %d", config.saturation_yellow_h)
        rospy.loginfo("lightness_yellow_l : %d", config.lightness_yellow_l)
        rospy.loginfo("lightness_yellow_h : %d", config.lightness_yellow_h)

        rospy.loginfo("hue_green_l : %d", config.hue_green_l)
        rospy.loginfo("hue_green_h : %d", config.hue_green_h)
        rospy.loginfo("saturation_green_l : %d", config.saturation_green_l)
        rospy.loginfo("saturation_green_h : %d", config.saturation_green_h)
        rospy.loginfo("lightness_green_l : %d", config.lightness_green_l)
        rospy.loginfo("lightness_green_h : %d", config.lightness_green_h)

        self.hue_red_l = config.hue_red_l
        self.hue_red_h = config.hue_red_h
        self.saturation_red_l = config.saturation_red_l
        self.saturation_red_h = config.saturation_red_h
        self.lightness_red_l = config.lightness_red_l
        self.lightness_red_h = config.lightness_red_h

        self.hue_yellow_l = config.hue_yellow_l
        self.hue_yellow_h = config.hue_yellow_h
        self.saturation_yellow_l = config.saturation_yellow_l
        self.saturation_yellow_h = config.saturation_yellow_h
        self.lightness_yellow_l = config.lightness_yellow_l
        self.lightness_yellow_h = config.lightness_yellow_h

        self.hue_green_l = config.hue_green_l
        self.hue_green_h = config.hue_green_h
        self.saturation_green_l = config.saturation_green_l
        self.saturation_green_h = config.saturation_green_h
        self.lightness_green_l = config.lightness_green_l
        self.lightness_green_h = config.lightness_green_h

        return config

    def cbGetImage(self, image_msg):

        #输入：图像信息
        #设置帧率，将图像转变成opencv格式
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        self.is_image_available = True

    def fnFindTrafficLight(self):

        #检测绿色信号灯，绿色计数加1；检测黄色信号灯，黄色计数加1；检测红色信号灯，较远距离，红色计数加1,较近距离，停止计数加1
        #根据各个状态计数值，发布max_vel,控制小车根据信号灯指示运动
        cv_image_mask = self.fnMaskGreenTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask, (5, 5), 0)

        cv2.namedWindow("window", 2)
        cv2.imshow("window", cv_image_mask)
        cv2.waitKey(3)

        status1 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'green')
        if status1 == 1 or status1 == 5:
            self.stop_count = 0
            self.green_count += 1
        else:
            self.green_count = 0

            cv_image_mask = self.fnMaskYellowTrafficLight()
            cv_image_mask = cv2.GaussianBlur(cv_image_mask, (5, 5), 0)

            status2 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'yellow')
            if status2 == 2:
                self.yellow_count += 1
            else:
                self.yellow_count = 0

                cv_image_mask = self.fnMaskRedTrafficLight()
                cv_image_mask = cv2.GaussianBlur(cv_image_mask, (5, 5), 0)

                status3 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'red')
                if status3 == 3:
                    self.red_count += 1
                elif status3 == 4:
                    self.red_count = 0
                    self.stop_count += 1
                else:
                    self.red_count = 0
                    self.stop_count = 0

        if self.green_count > 20:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("GREEN")
            cv2.putText(self.cv_image, "GREEN", (self.point_col, self.point_row), cv2.FONT_HERSHEY_DUPLEX, 0.5,
                        (80, 255, 0))

        if self.yellow_count > 12:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.06 if not self.off_traffic else 0.12
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("YELLOW")
            cv2.putText(self.cv_image, "YELLOW", (self.point_col, self.point_row), cv2.FONT_HERSHEY_DUPLEX, 0.5,
                        (0, 255, 255))

        if self.red_count > 8:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.03
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("RED")
            cv2.putText(self.cv_image, "RED", (self.point_col, self.point_row), cv2.FONT_HERSHEY_DUPLEX, 0.5,
                        (0, 0, 255))

        if self.stop_count > 8:
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.0
            self.pub_max_vel.publish(msg_pub_max_vel)
            rospy.loginfo("STOP")
            self.off_traffic = True
            cv2.putText(self.cv_image, "STOP", (self.point_col, self.point_row), cv2.FONT_HERSHEY_DUPLEX, 0.5,
                        (0, 0, 255))

        if self.pub_image_type == "compressed":
            # publishes traffic light image in compressed type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))

        elif self.pub_image_type == "raw":
            # publishes traffic light image in raw type
            self.pub_image_traffic_light.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

    def fnMaskRedTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_red_l
        Hue_h = self.hue_red_h
        Saturation_l = self.saturation_red_l
        Saturation_h = self.saturation_red_h
        Lightness_l = self.lightness_red_l
        Lightness_h = self.lightness_red_h

        # define range of red color in HSV
        lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_red = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask=mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes red light filtered image in compressed type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes red light filtered image in raw type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask

    def fnMaskYellowTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask=mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes yellow light filtered image in compressed type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes yellow light filtered image in raw type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask

    def fnMaskGreenTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_green_l
        Hue_h = self.hue_green_h
        Saturation_l = self.saturation_green_l
        Saturation_h = self.saturation_green_h
        Lightness_l = self.lightness_green_l
        Lightness_h = self.lightness_green_h

        # define range of green color in HSV
        lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_green = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask=mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes green light filtered image in compressed type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes green light filtered image in raw type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        cv2.namedWindow("window2", 2)
        cv2.imshow("window2", self.cv_image)
        cv2.waitKey(3)

        mask = cv2.bitwise_not(mask)

        return mask

    def fnFindCircleOfTrafficLight(self, mask, find_color):

        #输入：掩膜图像，指示灯颜色

        #斑点检测算法:
        #斑点：是一组连通的像素在图像中共享一些共同的属性（如灰度值、颜色等）
        #该算法通过参数创建过滤器过滤斑点：通过像素阈值过滤，通过斑点面积过滤，通过圆性（4*pi*area/perimeter**2）过滤
        #通过凸性(斑点面积/斑点凸包面积)_过滤,通过惯性比（Inertia Ratio）过滤，一个形状有多长（圆1,椭圆0-1.直线0）
        #将检测出的斑点用红色圆包围

        #设定区域范围（离信号灯的远近），求出斑点中心坐标，根据斑点是否在设定范围内以及信号灯颜色来指定5个状态status
        #输出：状态编号status

        status = 0

        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 600

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.6

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.6

        det = cv2.SimpleBlobDetector_create(params)
        keypts = det.detect(mask)
        frame = cv2.drawKeypoints(self.cv_image, keypts, np.array([]), (0, 0, 255),
                                  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.namedWindow("window3", 2)
        cv2.imshow("window3", frame)
        cv2.waitKey(3)

        print(len(keypts))

        col1 = 180
        col2 = 270
        col3 = 305

        row1 = 50
        row2 = 170
        row3 = 170

        # if detected more than 1 light
        for i in range(len(keypts)):
            self.point_col = int(keypts[i].pt[0])
            self.point_row = int(keypts[i].pt[1])

            print(self.point_col, self.point_row)

            if self.point_col > col1 and self.point_col < col2 and self.point_row > row1 and self.point_row < row2:
                if find_color == 'green':
                    status = 1
                elif find_color == 'yellow':
                    status = 2
                elif find_color == 'red':
                    status = 3
            elif self.point_col > col2 and self.point_col < col3 and self.point_row > row1 and self.point_row < row3:
                if find_color == 'red':
                    status = 4
                elif find_color == 'green':
                    status = 5
            else:
                status = 6

        return status

    def cbTrafficLightFinished(self, traffic_light_finished_msg):
        self.is_traffic_light_finished = True

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('detect_traffic_light_note')
    node = DetectTrafficLight()
    node.main()
