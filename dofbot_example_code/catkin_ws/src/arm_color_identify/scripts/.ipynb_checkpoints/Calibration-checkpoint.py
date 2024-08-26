# !/usr/bin/env python
# coding: utf-8
import random
import cv2 as cv
import numpy as np
import Arm_Lib

class Arm_Calibration:
    def __init__(self):
        # 初始化图像
        self.image = None
        self.threshold_num = 140
        # 机械臂识别位置调节
        self.xy = [90, 135]
        self.rect_init()
        self.index = 0
        self.GetPoints_status = "Runing"
        self.rect_A = (self.rect_init_x, self.rect_init_y)
        self.rect_B = (self.rect_end_x, self.rect_end_y)
        self.point_initX_list=[]
        self.point_initY_list=[]
        self.point_endX_list=[]
        self.point_endY_list=[]
        # 创建机械臂驱动实例
        self.arm = Arm_Lib.Arm_Device()

    def calibration_map(self, image, xy=None, threshold_num=140):
        '''
        放置方块区域检测函数
        :param image:输入图像
        :return:轮廓区域边点,处理后的图像
        '''
        if xy != None: self.xy = xy
        # 机械臂初始位置角度
        joints_init = [self.xy[0], self.xy[1], 0, 0, 90, 30]
        # 将机械臂移动到标定方框的状态
        self.arm.Arm_serial_servo_write6_array(joints_init, 1500)
        self.image = image
        self.threshold_num = threshold_num
        # 创建边点容器
        dp = []
        h, w = self.image.shape[:2]
        # 获取轮廓点集(坐标)
        contours = self.Morphological_processing()
        # 遍历点集
        for i, c in enumerate(contours):
            # 计算轮廓区域。
            area = cv.contourArea(c)
            # 设置轮廓区域范围
            if h * w / 2 < area < h * w:
                # 计算多边形的矩
                mm = cv.moments(c)
                if mm['m00'] == 0:
                    continue
                cx = mm['m10'] / mm['m00']
                cy = mm['m01'] / mm['m00']
                # 绘制轮廓区域
                cv.drawContours(self.image, contours, i, (255, 255, 0), 2)
                # 获取轮廓区域边点
                dp = np.squeeze(cv.approxPolyDP(c, 100, True))
                # 绘制中心
                cv.circle(self.image, (np.int32(cx), np.int32(cy)), 5, (0, 0, 255), -1)
        return dp, self.image

    def Morphological_processing(self):
        '''
        形态学及去噪处理,并获取轮廓点集
        '''
        # 将图像转为灰度图
        gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        # 使用高斯滤镜模糊图像。
        gray = cv.GaussianBlur(gray, (5, 5), 1)
        # 图像二值化操作
        ref, threshold = cv.threshold(gray, self.threshold_num, 255, cv.THRESH_BINARY)
        # 获取不同形状的结构元素
        kernel = np.ones((3, 3), np.uint8)
        # 形态学开操作
        blur = cv.morphologyEx(threshold, cv.MORPH_OPEN, kernel, iterations=4)
        # 提取模式
        mode = cv.RETR_EXTERNAL
        # 提取方法
        method = cv.CHAIN_APPROX_NONE
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        # 层级关系 参数一：输入的二值图，参数二：提取模式，参数三：提取方法。
        contours, hierarchy = cv.findContours(blur, mode, method)
        return contours

    def Perspective_transform(self, dp, image):
        '''
        透视变换
        :param dp: 方框边点(左上,左下,右下,右上)
        :param image: 原始图像
        :return: 透视变换后图像
        '''
        if len(dp) != 4: return
        upper_left = []
        lower_left = []
        lower_right = []
        upper_right = []
        for i in range(len(dp)):
            if dp[i][0] < 320 and dp[i][1] < 240:
                upper_left = dp[i]
            if dp[i][0] < 320 and dp[i][1] > 240:
                lower_left = dp[i]
            if dp[i][0] > 320 and dp[i][1] > 240:
                lower_right = dp[i]
            if dp[i][0] > 320 and dp[i][1] < 240:
                upper_right = dp[i]
        # 原图中的四个顶点
        pts1 = np.float32([upper_left, lower_left, lower_right, upper_right])
        # 变换后的四个顶点
        pts2 = np.float32([[0, 0], [0, 480], [640, 480], [640, 0]])
        # 根据四对对应点计算透视变换。
        M = cv.getPerspectiveTransform(pts1, pts2)
        # 将透视变换应用于图像。
        Transform_img = cv.warpPerspective(image, M, (640, 480))
        return Transform_img

    def get_points(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        value = 30
        move_value = 100
        min_point = (-1,-1)
        max_point = (-1,-1)
        points = [[0, 0], [0, 0], [0, 0], [0, 0]]
        try:
            """
            maxCorners 最大角点数
            qualityLevel 最低接受质量百分比
            minDistance 点之间的最小距离
            """
            corners = cv.goodFeaturesToTrack(gray, 4, 0.3, 10)
            if len(corners) == 4:
                corners = np.int0(corners)
                for i in range(4):
                    points[i][0] = corners[i][0][0]
                    points[i][1] = corners[i][0][1]
            min_P = min(points[0][0] + points[0][1],
                           points[1][0] + points[1][1],
                           points[2][0] + points[2][1],
                           points[3][0] + points[3][1])
            max_P = max(points[0][0] + points[0][1],
                           points[1][0] + points[1][1],
                           points[2][0] + points[2][1],
                           points[3][0] + points[3][1])
            for i in range(4):
                if (points[i][0] + points[i][1]) == min_P: min_point = points[i]
                if (points[i][0] + points[i][1]) == max_P: max_point = points[i]
            point_initX = min_point[0] + value
            point_initY = min_point[1] + value - move_value
            point_endX = max_point[0] - value
            point_endY = max_point[1] - value - move_value
            Area = (point_endX - point_initX) * (point_endY - point_initY)
            if Area <= 7000:
                point_A = (point_initX, point_initY)
                point_B = (point_endX, point_endY)
                cv.rectangle(img, point_A, point_B, (0, 255, 0), 2)
                if point_initX>0: self.point_initX_list.append(point_initX)
                if point_initY>0: self.point_initY_list.append(point_initY)
                if point_endX>0: self.point_endX_list.append(point_endX)
                if point_endY>0: self.point_endY_list.append(point_endY)
        except Exception:
            pass
        return img

    def get_hsv(self, img):

        '''
        获取某一区域的HSV的范围
        :param img: 彩色图
        :return: 图像和HSV的范围
        '''
        img = cv.resize(img, (640, 480), )
        # 设置识别区域
        point_Xmin = 50  # 200
        point_Xmax = 600  # 440
        point_Ymin = 80  # 200
        point_Ymax = 480  # 480
        # 画矩形框
        # cv.rectangle(self.image, (point_Xmin, point_Ymin), (point_Xmax, point_Ymax),(105,105,105), 2)
        # 复制原始图像,避免处理过程中干扰
        img = img[point_Ymin:point_Ymax, point_Xmin:point_Xmax]
        img = cv.resize(img, (640, 480))
        if self.index <= 50:
            hsv_range = ((0, 43, 46), (255, 255, 255))
            self.index += 1
            return img, hsv_range
        if self.GetPoints_status == "Runing" and 50 < self.index < 150:
            img = self.get_points(img)
            hsv_range = ((0, 43, 46), (255, 255, 255))
            self.index += 1
            return img, hsv_range
        if self.index >= 150:
            # 获取平铺后每个索引位置值在原始数列中出现的次数
            if len(self.point_initX_list)!=0 and \
                    len(self.point_initY_list)!=0 and  \
                    len(self.point_endX_list)!=0 and  \
                    len(self.point_endY_list)!=0: self.set_rect()
#             print("self.rect_A:",self.rect_A)
#             print("self.rect_B:",self.rect_B)
            self.GetPoints_status = "waiting"
            self.index += 1
        if self.GetPoints_status == "waiting":
            img, hsv_range = self.Read_HSV(img)
            return img, hsv_range

    def set_rect(self):
        initX_list = np.argmax(np.bincount(self.point_initX_list))
        initY_list = np.argmax(np.bincount(self.point_initY_list))
        endX_list = np.argmax(np.bincount(self.point_endX_list))
        endY_list = np.argmax(np.bincount(self.point_endY_list))
        self.rect_A = (initX_list, initY_list)
        self.rect_B = (endX_list, endY_list)

    def set_index(self):
        self.index = 0
        self.GetPoints_status = "Runing"
        self.point_initX_list=[]
        self.point_initY_list=[]
        self.point_endX_list=[]
        self.point_endY_list=[]
        
    def rect_init(self):
        # 初始化矩形框
        self.rect_init_x = 290
        self.rect_init_y = 300
        self.rect_end_x = 350
        self.rect_end_y = 360

    def Read_HSV(self, img):
        H = [];S = [];V = []
        if sum(self.rect_A)<sum(self.rect_B):
            (a, b) = self.rect_A
            (c, d) = self.rect_B
        else:
            (a, b) = self.rect_B
            (c, d) = self.rect_A
        self.rect_init_x = min(a, c)
        self.rect_end_x = max(a, c)
        self.rect_init_y = min(b, d)
        self.rect_end_y = max(b, d)
        if self.rect_init_x <= 0: self.rect_init()
        if self.rect_init_y <= 0: self.rect_init()
        if self.rect_end_x >= 640: self.rect_init()
        if self.rect_end_y >= 480: self.rect_init()
        # 将彩色图转成HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        cv.rectangle(img, (self.rect_init_x, self.rect_init_y), (self.rect_end_x, self.rect_end_y), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中
        for i in range(self.rect_init_x, self.rect_end_x):
            for j in range(self.rect_init_y, self.rect_end_y):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # 分别计算出H,S,V的最大最小
        H_min = min(H);
        H_max = max(H)
        S_min = min(S);
        S_max = max(S)
        V_min = min(V);
        V_max = max(V)
        # HSV范围调整
        if H_min - 2 < 0:H_min = 0
        else:H_min -= 2
        if S_min - 15 < 0:S_min = 0
        else:S_min -= 15
        if V_min - 15 < 0:V_min = 0
        else:V_min -= 15
        if H_max + 2 > 255:H_max = 255
        else:H_max += 2
        S_max = 253;V_max = 255
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:
            cv.putText(img, txt1, (self.rect_init_x - 15, self.rect_init_y - 15), cv.FONT_HERSHEY_SIMPLEX, 1,
                       (0, 0, 255), 1)
        else:
            cv.putText(img, txt2, (self.rect_init_x - 15, self.rect_init_y - 15), cv.FONT_HERSHEY_SIMPLEX, 1,
                       (0, 255, 0), 1)
        cv.putText(img, lowerb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(img, upperb, (150, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range

class update_hsv:
    def __init__(self):
        '''
        初始化一些参数
        '''
        self.image = None
        self.binary = None

    def Image_Processing(self, hsv_range):
        '''
        形态学变换去出细小的干扰因素
        :param img: 输入初始图像
        :return: 检测的轮廓点集(坐标)
        '''
        (lowerb, upperb) = hsv_range
        # 复制原始图像,避免处理过程中干扰
        color_mask = self.image.copy()
        # 将图像转换为HSV。
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # 筛选出位于两个数组之间的元素。
        color = cv.inRange(hsv_img, lowerb, upperb)
        # 设置非掩码检测部分全为黑色
        color_mask[color == 0] = [0, 0, 0]
        # 将图像转为灰度图
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标) python2和python3在此处略有不同
        # _, contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) #python2
        contours, heriachy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)  # python3
        return contours, binary

    def draw_contours(self, hsv_name, contours):
        '''
        绘制轮廓
        '''
        for i, cnt in enumerate(contours):
            # 计算多边形的矩
            mm = cv.moments(cnt)
            if mm['m00'] == 0:
                continue
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            # ⾯积限制
            if area > 800:
                # 获取多边形的中心
                (x, y) = (np.int32(cx), np.int32(cy))
                # 绘制中⼼
                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)
                # 计算最小矩形区域
                rect = cv.minAreaRect(cnt)
                # 获取盒⼦顶点
                box = cv.boxPoints(rect)
                # 转成long类型
                box = np.int64(box)
                # 绘制最小矩形
                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                cv.putText(self.image, hsv_name, (int(x - 15), int(y - 15)),
                           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

    def get_contours(self, img, color_name, hsv_msg, color_hsv):
        binary = None
        # 规范输入图像大小
        self.image = cv.resize(img, (640, 480), )
        for key, value in color_hsv.items():
            # 检测轮廓点集
            if color_name == key:
                color_contours, binary = self.Image_Processing(hsv_msg)
            else:
                color_contours, _ = self.Image_Processing(color_hsv[key])
            # 绘制检测图像,并控制跟随
            self.draw_contours(key, color_contours)
        return self.image, binary


