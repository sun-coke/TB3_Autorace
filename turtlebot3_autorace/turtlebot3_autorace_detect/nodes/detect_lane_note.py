#!/usr/bin/env python
# -*- coding:utf8 -*-

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server
from turtlebot3_autorace_detect.cfg import DetectLaneParamsConfig


class DetectLane():
    def __init__(self):
        self.hue_white_l = rospy.get_param("~detect/lane/white/hue_l", 0)
        self.hue_white_h = rospy.get_param("~detect/lane/white/hue_h", 25)
        self.saturation_white_l = rospy.get_param("~detect/lane/white/saturation_l", 0)
        self.saturation_white_h = rospy.get_param("~detect/lane/white/saturation_h", 36)
        self.lightness_white_l = rospy.get_param("~detect/lane/white/lightness_l", 180)
        self.lightness_white_h = rospy.get_param("~detect/lane/white/lightness_h", 255)

        self.hue_yellow_l = rospy.get_param("~detect/lane/yellow/hue_l", 27)
        self.hue_yellow_h = rospy.get_param("~detect/lane/yellow/hue_h", 41)
        self.saturation_yellow_l = rospy.get_param("~detect/lane/yellow/saturation_l", 130)
        self.saturation_yellow_h = rospy.get_param("~detect/lane/yellow/saturation_h", 255)
        self.lightness_yellow_l = rospy.get_param("~detect/lane/yellow/lightness_l", 160)
        self.lightness_yellow_h = rospy.get_param("~detect/lane/yellow/lightness_h", 255)

        self.is_calibration_mode = rospy.get_param("~is_detection_calibration_mode", False)
        if self.is_calibration_mode == True:
            srv_detect_lane = Server(DetectLaneParamsConfig, self.cbGetDetectLaneParam)

        self.sub_image_type = "raw"  # you can choose image type "compressed", "raw"
        self.pub_image_type = "compressed"  # you can choose image type "compressed", "raw"

        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage,
                                                       self.cbFindLane, queue_size=1)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/camera/image_projected_compensated', Image, self.cbFindLane, queue_size=1)

        if self.pub_image_type == "compressed":
            # publishes lane image in compressed type
            self.pub_image_lane = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size=1)
        elif self.pub_image_type == "raw":
            # publishes lane image in raw type
            self.pub_image_lane = rospy.Publisher('/detect/image_output', Image, queue_size=1)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes lane image in compressed type
                self.pub_image_white_lane = rospy.Publisher('/detect/image_output_sub1/compressed', CompressedImage,
                                                            queue_size=1)
                self.pub_image_yellow_lane = rospy.Publisher('/detect/image_output_sub2/compressed', CompressedImage,
                                                             queue_size=1)
            elif self.pub_image_type == "raw":
                # publishes lane image in raw type
                self.pub_image_white_lane = rospy.Publisher('/detect/image_output_sub1', Image, queue_size=1)
                self.pub_image_yellow_lane = rospy.Publisher('/detect/image_output_sub2', Image, queue_size=1)

        self.pub_lane = rospy.Publisher('/detect/lane', Float64, queue_size=1)

        # subscribes state : yellow line reliability
        self.pub_yellow_line_reliability = rospy.Publisher('/detect/yellow_line_reliability', UInt8, queue_size=1)

        # subscribes state : white line reliability
        self.pub_white_line_reliability = rospy.Publisher('/detect/white_line_reliability', UInt8, queue_size=1)

        self.cvBridge = CvBridge()

        self.counter = 1

        self.window_width = 1000.
        self.window_height = 600.

        self.reliability_white_line = 100
        self.reliability_yellow_line = 100

    def cbGetDetectLaneParam(self, config, level):
        rospy.loginfo("[Detect Lane] Detect Lane Calibration Parameter reconfigured to")
        rospy.loginfo("hue_white_l : %d", config.hue_white_l)
        rospy.loginfo("hue_white_h : %d", config.hue_white_h)
        rospy.loginfo("saturation_white_l : %d", config.saturation_white_l)
        rospy.loginfo("saturation_white_h : %d", config.saturation_white_h)
        rospy.loginfo("lightness_white_l : %d", config.lightness_white_l)
        rospy.loginfo("lightness_white_h : %d", config.lightness_white_h)
        rospy.loginfo("hue_yellow_l : %d", config.hue_yellow_l)
        rospy.loginfo("hue_yellow_h : %d", config.hue_yellow_h)
        rospy.loginfo("saturation_yellow_l : %d", config.saturation_yellow_l)
        rospy.loginfo("saturation_yellow_h : %d", config.saturation_yellow_h)
        rospy.loginfo("lightness_yellow_l : %d", config.lightness_yellow_l)
        rospy.loginfo("lightness_yellow_h : %d", config.lightness_yellow_h)

        self.hue_white_l = config.hue_white_l
        self.hue_white_h = config.hue_white_h
        self.saturation_white_l = config.saturation_white_l
        self.saturation_white_h = config.saturation_white_h
        self.lightness_white_l = config.lightness_white_l
        self.lightness_white_h = config.lightness_white_h

        self.hue_yellow_l = config.hue_yellow_l
        self.hue_yellow_h = config.hue_yellow_h
        self.saturation_yellow_l = config.saturation_yellow_l
        self.saturation_yellow_h = config.saturation_yellow_h
        self.lightness_yellow_l = config.lightness_yellow_l
        self.lightness_yellow_h = config.lightness_yellow_h

        return config

    def cbFindLane(self, image_msg):
        # TODO: mov_avg error

        # Change the frame rate by yourself. Now, it is set to 1/3 (10fps).
        # Unappropriate value of frame rate may cause huge delay on entire recognition process.
        # This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == "compressed":
            # converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        # find White and Yellow Lanes
        white_fraction, cv_white_lane = self.maskWhiteLane(cv_image)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_image)

        print(white_fraction, yellow_fraction)

        cv_lane = cv2.addWeighted(cv_white_lane, 1, cv_yellow_lane, 1, 0)

        cv2.namedWindow("window", 2)
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)


        cv2.namedWindow("window2", 2)
        cv2.imshow("window2", cv_lane)
        cv2.waitKey(3)


        try:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, cv_yellow_lane)
                self.mov_avg_left = np.append(self.mov_avg_left, np.array([self.left_fit]), axis=0)


            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, cv_white_lane)
                self.mov_avg_right = np.append(self.mov_avg_right, np.array([self.right_fit]), axis=0)

        except:
            if yellow_fraction > 3000:
                yellow_x,yellow_y, self.left_fitx, self.left_fit, win_yellow, ploty = self.sliding_windown(cv_yellow_lane, 'left')
                self.mov_avg_left = np.array([self.left_fit])

                win_yellow[yellow_y, yellow_x] = [0,0,255]
                #print(ploty, self.left_fitx)

            if white_fraction > 3000:
                white_x, white_y, self.right_fitx, self.right_fit, win_white, ploty = self.sliding_windown(cv_white_lane, 'right')
                self.mov_avg_right = np.array([self.right_fit])

                win_white[white_y, white_x] = [255,255,0]
                #print(ploty, self.right_fitx)

                win_lane = cv2.addWeighted(win_yellow, 1, win_white, 1, 0)

                cv2.namedWindow("window3", 2)
                cv2.imshow("window3", win_lane)
                cv2.waitKey(3)

        #print(self.mov_avg_right.shape[1])

        MOV_AVG_LENGTH = 5

        self.left_fit = np.array([np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                  np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                  np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])])
        self.right_fit = np.array([np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                   np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                   np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])])

        if self.mov_avg_left.shape[0] > 1000:
            self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

        if self.mov_avg_right.shape[0] > 1000:
            self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]

        self.make_lane(cv_image, white_fraction, yellow_fraction)

    def maskWhiteLane(self, image):

        #输入：透视变换后的图像
        #将BGR图像转化成HSV图像，设定阈值对图像进掩膜操作，提取出白线
        #计算出mask图中的非零像素点的个数fraction_num，根据非零像素点的个数实时调整亮度阈值
        #计算非零像素行的个数，根据非零像素行的个数实时发布白色可靠线的值
        #返回：非零像素点的个数和提取出的白线0-1图像

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_white_l
        Hue_h = self.hue_white_h
        Saturation_l = self.saturation_white_l
        Saturation_h = self.saturation_white_h
        Lightness_l = self.lightness_white_l
        Lightness_h = self.lightness_white_h

        # define range of white color in HSV
        lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask=mask)

        fraction_num = np.count_nonzero(mask)

        if self.is_calibration_mode == False:
            if fraction_num > 35000:
                if self.lightness_white_l < 250:
                    self.lightness_white_l += 5
            elif fraction_num < 5000:
                if self.lightness_white_l > 50:
                    self.lightness_white_l -= 5

        how_much_short = 0

        for i in range(0, 600):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1

        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_white_line >= 5:
                self.reliability_white_line -= 5
        elif how_much_short <= 100:
            if self.reliability_white_line <= 99:
                self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg_white_line_reliability)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes white lane filtered image in compressed type
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes white lane filtered image in raw type
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, "bgr8"))

        return fraction_num, mask

    def maskYellowLane(self, image):
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

        fraction_num = np.count_nonzero(mask)

        if self.is_calibration_mode == False:
            if fraction_num > 35000:
                if self.lightness_yellow_l < 250:
                    self.lightness_yellow_l += 20
            elif fraction_num < 5000:
                if self.lightness_yellow_l > 90:
                    self.lightness_yellow_l -= 20

        how_much_short = 0

        for i in range(0, 600):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1

        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_yellow_line >= 5:
                self.reliability_yellow_line -= 5
        elif how_much_short <= 100:
            if self.reliability_yellow_line <= 99:
                self.reliability_yellow_line += 5

        msg_yellow_line_reliability = UInt8()
        msg_yellow_line_reliability.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes yellow lane filtered image in compressed type
                self.pub_image_yellow_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes yellow lane filtered image in raw type
                self.pub_image_yellow_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, "bgr8"))

        return fraction_num, mask

    def fit_from_lines(self, lane_fit, image):

        #输入：滑窗帧的系数值，车道线的0-1图像
        #视频数据是连续的图像信息，基于连续两帧图像不会突变的先验知识，利用上一帧的检测结果作为下一帧的输入，检测上一帧结果周围的点，可以减少计算量并保证准确性
        #输出：二次多项式和系数值

        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) & (
                nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin)))

        # Again, extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        lane_fit = np.polyfit(y, x, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):

        #输入：提取出的白/黄线的0-1图，左/右
        #为在有噪点的图像中定位车道线的起始位置，对下半部分图像在每一列上将白色像素点个数做和形成直方图，取最大值的横坐标作为车道线起始位置
        #设定滑窗个数，计算滑窗高度，设定范围margin值
        #计算滑窗的边界坐标
        #存储滑窗中非零像素点的横坐标索引，如果滑窗中的像素点超过设定的最小值，取滑窗中像素点的均值更新下一个滑窗的中心位置，以此往复，直到把所有行都搜索完
        #将存储的索引值转化成opencv格式，提取像素线的位置坐标，利用二次多项式进行拟合。生成系数值（lane_fit）以及二次多项式（lane_fitx）
        #返回：二次多项式和系数值

        histogram = np.sum(img_w[img_w.shape[0] / 2:, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 20

        # Set height of windows
        window_height = np.int(img_w.shape[0] / nwindows)

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated for each window
        x_current = lane_base

        # Set the width of the windows +/- margin
        margin = 50

        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Create empty lists to receive lane pixel indices
        lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            # Identify the nonzero pixels in x and y within the window
            good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                    nonzerox < win_x_high)).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_lane_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_lane_inds) > minpix:
                x_current = np.int(np.mean(nonzerox[good_lane_inds]))

        # Concatenate the arrays of indices
        lane_inds = np.concatenate(lane_inds)

        # Extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except:
            lane_fit = self.lane_fit_bef

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return x, y, lane_fitx, lane_fit, out_img, ploty

    def make_lane(self, cv_image, white_fraction, yellow_fraction):

        #输入：透视原图，提取车道的像素点数
        #将左右车道线二次多项式的横坐标转换成opencv格式，求出点对绘制出左右车道线
        #利用左右车道线像素点横坐标取均值求出中点横坐标，将格式转换成pence格式求出中心坐标点对绘制中心线
        #填充车道线中间区域
        #将中心线和ROI区域叠加到原图上，输出final
        #输出：发布中心线横坐标和final图像信息

        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)

        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        if yellow_fraction > 3000:
            #将点对格式重新转换成cv2.polylines处理的格式
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            #vstack()数组垂直合并;transpose()矩阵转置;flipud()矩阵翻转
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=30)
            #ploylines()多边形绘制。pts点对；isClosed：True表示的是线段闭合，False表示的是仅保留线段；
            #thickness：数值型，厚度，默认值为1，如果对封闭图形，正方形，三角形等传入-1,则会填充整个图形。
            cv2.namedWindow("window4", 2)
            cv2.imshow("window4", color_warp_lines)
            cv2.waitKey(3)

        if white_fraction > 3000:
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=30)

            cv2.namedWindow("window4", 2)
            cv2.imshow("window4", color_warp_lines)
            cv2.waitKey(3)

        self.is_center_x_exist = True

        if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:
            if white_fraction > 3000 and yellow_fraction > 3000:
                centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
                pts = np.hstack((pts_left, pts_right))
                #格式转换
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255),
                              thickness=12)
                #print(centerx)
                cv2.namedWindow("window4", 2)
                cv2.imshow("window4", color_warp_lines)
                cv2.waitKey(3)

                # Draw the lane onto the warped blank image
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

            if white_fraction > 3000 and yellow_fraction <= 3000:
                centerx = np.subtract(self.right_fitx, 320)
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255),
                              thickness=12)

                cv2.namedWindow("window4", 2)
                cv2.imshow("window4", color_warp_lines)
                cv2.waitKey(3)

            if white_fraction <= 3000 and yellow_fraction > 3000:
                centerx = np.add(self.left_fitx, 320)
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255),
                              thickness=12)

                cv2.namedWindow("window4", 2)
                cv2.imshow("window4", color_warp_lines)
                cv2.waitKey(3)

        elif self.reliability_white_line <= 50 and self.reliability_yellow_line > 50:
            centerx = np.add(self.left_fitx, 320)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

            cv2.namedWindow("window4", 2)
            cv2.imshow("window4", color_warp_lines)
            cv2.waitKey(3)

        elif self.reliability_white_line > 50 and self.reliability_yellow_line <= 50:
            centerx = np.subtract(self.right_fitx, 320)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

            cv2.namedWindow("window4", 2)
            cv2.imshow("window4", color_warp_lines)
            cv2.waitKey(3)

        else:
            self.is_center_x_exist = False
            # TODO: stop
            pass

        # Combine the result with the original image
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        cv2.namedWindow("window5", 2)
        cv2.imshow("window5", final)
        cv2.waitKey(3)

        if self.pub_image_type == "compressed":
            if self.is_center_x_exist == True:
                # publishes lane center
                msg_desired_center = Float64()
                msg_desired_center.data = centerx.item(350)
                self.pub_lane.publish(msg_desired_center)

            self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, "jpg"))

        elif self.pub_image_type == "raw":
            if self.is_center_x_exist == True:
                # publishes lane center
                msg_desired_center = Float64()
                msg_desired_center.data = centerx.item(350)
                self.pub_lane.publish(msg_desired_center)

            self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, "bgr8"))

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('detect_lane_note')
    node = DetectLane()
    node.main()
