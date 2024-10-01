#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Usage: roslaunch lidar_avoidance_final_ref lidar_avoidance_final_ref.launch
# ROS node for lidar avoidance
#==============================================================================

# BEGIN ALL
import rospy
import os
import cv2
import cv_bridge
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from traffic_light_classifier.msg import traffic_light

global green_cnt

green_cnt = 0

err1_prev = 0 
err_prev = 0

h = 144
w = 192
w_mid = 110 # 120일 때 선을 거의 중앙에 둠, 근데 그럼 좌우 턴 최대값이 달라질 수 있음

roi_down = 144 - 28     # 20%
roi_up = 144 - 35       # 25%
roi_mid = 96

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.traffic_sub = rospy.Subscriber("/light_color", traffic_light, self.traffic_callback)
        self.lidar_sub = rospy.Subscriber("/scan_raw", LaserScan, self.lidar_callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/lane_image", Image, queue_size=1)
        self.image_debug_pub = rospy.Publisher("/lane_debug_image", Image, queue_size=1)

        self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)
        self.roi_center_pix = 0 # 1~3 index will be used

        self.rngL = None
        self.rngR = None
        self.rngM = None
        self.traffic_color = 0

        ###------------------ 파라미터 수정 -----------------------###
        ##@@ 실행 모드
        self.debug = False

        ##@@ 라인검출 색 범위
        self.lower_red = numpy.array([143,75,53])
        self.upper_red = numpy.array([202,255,255])
        self.lower_red2 = numpy.array([0,75,53])
        self.upper_red2 = numpy.array([10,255,255])

        # ! linear.x, angular.z는 cilab_driver에서 1000이 곱해져 register값으로 적용된다. 0.001까지 유효하다.
        ##@@ 제어 파라미터
        # main
        self.base_K = 0.004  #0004
        self.slow_K = 0.004  #0006
        '''
        sp = 1 일 때, K = 0.010이면 와블링된다. 
        sp = 1 일 때, K = 0.007이면 good

        sp = 1 일 때, K = 0.007, D = 0.001 very good, 변환에 조금 둔하긴 한데 overshoot거의 없음
        sp = 1 일 때, K = 0.008, D = 0.001 very good, 반응성이 좋아지는 대신 변화 후 초반 진동이 있다.
        sp = 1 일 때, K = 0.007, D = 0.003 very good, 변환에서 리버스없고 진동 잘잡는 듯
        '''
        self.base_D = 0.007 #0007
        self.slow_D = 0.007 #0002
        '''
        #slow_D는 주변에 물체를 인식했을 떄 적용되는 D항인데
        #물체가 offset 범위에 들어왔을 때 급격하게 변하는 err 때문에
        #D항이 크면 오히려 물체쪽으로 움직여버린다. 따라서 slow_D는 오히려 base_D보다 작아야 한다.
        '''
        # self.base_sp = 0
        # self.slow_sp = 0

        
        # @ 장애물 작동 유무 실험(라이다 각도 조정) -> 
        # self.base_sp = 1.5
        # self.slow_sp = 0.9 # test 1.5

        # @ 주행 속도 증가 실험 -> 오류 시 base_K, base_D 조절 -> 원복 base 0.004, 0.007 게인
        # self.base_sp = 1.7
        # self.slow_sp = 1.0

        self.base_sp = 2.0
        self.slow_sp = 2.0

        # @ 장애물 속도 증가 실험 -> 원복 slow 0.006, 0.002 게인
        # self.base_sp = 1.7
        # self.slow_sp = 1.5


        self.K = self.base_K
        self.D = self.base_D
        self.speed = self.base_sp
        self.obj_in = False
        self.left_out = True
        self.right_out = True

        #front에서 물체 감지 범위(in)
        self.rngM_deg = 40
        self.rngM_thres =0.80 # offset 결정 범위와 같아야 함. 아니면 순간적으로 sp=2의 이상한 K,D값이 적용된 구간이 생겨 와블링됨. 
        # self.obj_offset1 = 0#100

        # left에서 물체 감지 범위(out)
        self.rng_deg_L = -120
        self.rng_thres_L = 0.80 # 최소 0.80이어야 반응 할 수 있다.
        self.obj_offset_L = 20 # 원복30
        ###======================================================###


    ## : /light_color 토픽에 따른 콜백함수 - 신호판별 node의 신호판단값 저장
    def traffic_callback(self, msg):
        global green_cnt
        self.traffic_color = msg.recognition_result

        if self.traffic_color == 1:
            green_cnt += 1
            print(green_cnt)

    ## : /scan_raw 토픽에 따른 콜백함수 - lidar값 저장
    '''
    msg는 lidar 데이터를 msg.ranges 멤버변수에 720길이의 list로 저장하고 있다. 값의 단위는 'm'다. 거리값은 꽤 정확하다.
    앞를 향하고 있는 cilabrobot을 위에서 볼 때, 
    12시방향 lidar값은 msg.ranges[0],
    9시방향 lidar값은 msg.ranges[180],
    6시방향 lidar값은 msg.ranges[360],
    3시방향 lidar값은 msg.ranges[540],
    인덱스마다 0.5도 증가 된다.
    c.f) cilabrobot 케이스는 너무 매끈해서 물체 인식이 안됨. lidar 적외선 반사되는 듯
    '''
    def lidar_callback(self, msg):
        # static offset
        # angles = [x for x in range(0, -90, -5)]  # [0, -5 , -10, ... , -85] : 12시에서 CW로 0~90도 사이 물체 탐지
        self.rngL = [msg.ranges[x * 2] for x in range(0, self.rng_deg_L, -3)]
        # self.rngL = [round(x,2) for x in self.rngL]
        # print(self.rngL)
        self.rngM = [msg.ranges[x * 2] for x in range(-self.rngM_deg, self.rngM_deg, 3)]
        
    

    ## : /usb_cam/image_raw 토픽에 따른 콜백함수 - line tracking 실행
    def image_callback(self, msg):
        global perr, ptime, serr, dt
        global err1_prev, err_prev

        # ------ take image and transform to hsv ------ #
        image0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = cv2.resize(image0, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_CUBIC)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # ============================================= #

        # ------------ extract red line -------------- #
        maskr = cv2.inRange(hsv, self.lower_red, self.upper_red)
        maskr2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        line_img = cv2.bitwise_or(maskr2, maskr)
        # ============================================= #

        # ------------ red obj noise filter ----------- #
        kernel = numpy.ones((3, 3), numpy.uint8)
        line_img = cv2.morphologyEx(line_img, cv2.MORPH_ERODE, kernel, iterations=1, borderType=cv2.BORDER_CONSTANT)
        line_img = cv2.morphologyEx(line_img, cv2.MORPH_DILATE, kernel, iterations=2, borderType=cv2.BORDER_CONSTANT)
        # ============================================= #

        # ------------- only line detector ------------ #
        line_img[140:h, 0:w] = 255
        _, line_img = cv2.connectedComponents(line_img)
        line_img = numpy.uint8(numpy.where(line_img == line_img[142,96], 255, 0)) # 5%
        # ============================================= #

        # --------------------------- 영역 정보 ---------------------------------------- #
        # ! 화면의 가장 아래는 차 범퍼로부터 11cm 떨어진 부분이다.
        # ! 1m 떨어진 바닥은 h*0.65 위치의 화면에 잡힌다. 이 위로는 관객이나 기타 사물에 영향을 받을 수 있다.

        # ============================================================================= #

        # ----------------------- calculate offset -------------------------- #
        # ! lidar가 물체를 인식하는 순간 부터 처음으로 인식되는데 0.25~0.3초 정도 걸리고
        # ! 바퀴가 중앙정렬에서 서보모터로 offset값 50을 완전히변경하는데 0.15초 정도가 걸린다.
        
        # ! 실험+ : 물체옆을 지나가고 0.3초후에 바퀴가 다시 라인쪽으로 바라보기 시작한다. 
        
        
        if self.rngM != None:
            lidar_count = 0
            for d in self.rngM:
                if d < self.rngM_thres: 
                    lidar_count += 1

            if lidar_count >= 1:
                self.K = self.slow_K
                self.D = self.slow_D
                self.speed = self.slow_sp
                self.obj_in = True

        roi_offset = 0
        if self.rngL != None and self.obj_in:
            lidar_count = 0
            for d in self.rngL:
                if d < self.rng_thres_L: 
                    lidar_count += 1

            if lidar_count >= 1:
                roi_offset += self.obj_offset_L
                # self.left_out = False
            else :
                self.K = self.base_K
                self.D = self.base_D
                self.speed = self.base_sp
                self.obj_in = False
        else:
            roi_offset += 0


        # print(roi_offset)
        # ============================================================================= #


        # ----------------------- calculate error -------------------------- #
        roi = line_img[roi_up:roi_down, 0:w]
        self.calc_center_point_idxavr(roi, 10)
        err = self.roi_center_pix - roi_offset - w_mid
        derr = err - err_prev
        # ==================================================================== #


        # ------------------------ debug image make -------------------------- #
        if self.debug:
            cv2.circle(img, (w_mid, roi_mid), 2, (0, 255, 255), 2) 
            cv2.putText(img, str(err), (w_mid, roi_mid), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,255), 1)
            cv2.circle(img, (self.roi_center_pix, roi_mid), 3, (0, 255, 0), 2) 
            cv2.circle(img, (self.roi_center_pix - roi_offset, roi_mid), 6, (255, 0, 0), 2)

            self.image_debug_pub.publish(self.bridge.cv2_to_imgmsg(img[0:h, 0:w],encoding='bgr8'))
        # ==================================================================== #

        # ------------------ calculate velocity, angle ----------------------- #
        # @@ 조향값 범위 : -0.6~0.6, 속도값 범위 : -2.0~2.0
        # @@ error값은 -90~90 정도 범위를 가짐, 즉, K제어만 할 때는 K값이 0.007~0.006일 때 풍부한 제어

        ''' TUNING RECORD
        ang = err1 * self.K1 + derr1 * self.D1 : 
        sp=0.5, K1=0.007 no swing, very good
        sp=0.7, K1=0.007 little swing
        sp=0.8, K1=0.007 basic swing
        sp=1.0, K1=0.007 big swing

        sp=1.0, K1=0.004 커스텀코스1 못따라감, 대회코스는 따라갈 듯
        sp=1.0, K1=0.004 대회코스 곡률는 따라갈 듯, 지름 135cm정도의 곡선 가능, little swing
        sp=1.5, K1=0.004, swing 커짐.
        sp=1.0, K1=0.006  a lot swing
        sp=1.0, K1=0.006 D1=0.003  a little swing
        sp=1.0, K1=0.006 D1=0.006 little swing 
        sp=1.0, K1=0.006 D1=0.01  almost no swing !!
        sp=1.0, K1=0.006 D1=0.02  basic swing
        sp=1.0, K1=0.008 D1=0.01  big swing
        sp=1.0, K1=0.007 D1=0.012  basic swing
        sp=1.0, K1=0.007 D1=0.015  basic swing
        sp=1.0, K1=0.005 D1=0.008  no swing !!!

        sp=1.5, K1=0.004 D1=0.010  no swing !!! , 장애물 낫벧

        K1=0.003이하로 할 시 135cm 지름 턴 문제있음
        sp=1.8, K1=0.004 D1=0.010  basic swing 
        sp=1.8, K1=0.004 D1=0.013  basic swing
        sp=1.8, K1=0.004 D1=0.016  a lot swing
        sp=1.8, K1=0.004 D1=0.012  basic swing
        sp=1.8, K1=0.004 D1=0.008  basic swing

        sp=1.5, K1=0.004 D1=0.010  little swing
        sp=1.5, K1=0.004 D1=0.012  little swing

        sp=1.8, K1=0.004 D1=0.012  basic swing
        sp=2, K1=0.004 D1=0.012  basic swing
        sp=2, K1=0.004 D1=0.015  basic- swing
        sp=2, K1=0.004 D1=0.020  big swing
        sp=2, K1=0.004 D1=0.014  basic- swing

        ang = err * self.K2 + derr * self.D2 : 
        sp=2, K2=0.004 D2=0.014  little swing
        sp=2, K2=0.004 D2=0.018  little swing but crash
        sp=2, K2=0.004 D2=0.012  almost no swing
        sp=2, K2=0.004 D2=0.010  no swing !!!
        sp=2, K2=0.004 D2=0.008  no swing at all !!!!!!
        sp=2, K2=0.004 D2=0.006  no swing at all !!!!!!
        sp=2, K2=0.004 D2=0.001  a lot swing 
        sp=2, K2=0.004 D2=0.004  little swing 
        sp=2, K2=0.004 D2=0.007  no swing at all !!!!!! - perfect 
        sp=2, K2=0.005 D2=0.007  no swing !!!!!! - good
        sp=2, K2=0.006 D2=0.007  little swing
        sp=2, K2=0.006 D2=0.008  basic swing
        sp=2, K2=0.006 D2=0.006  basic swing
        
        sp=2, K2=0.0055 D2=0.007  no swing at all !!!!!! - perfect 
        K가 높을 수록 반응성이 좋기 떄문에 와블링이 되지 않는 선에서 최대한 올리는 것이 좋다.

        '''
        # print(green_cnt)
        if green_cnt >= 5:
        # if True:
            ang = err * self.K + derr * self.D

            self.twist.linear.x = self.speed
            self.twist.angular.z = -ang
            # print(self.K)
            # print(ang)
    
        # ==================================================================== #
        

        # ------------------------ message publish --------------------------- #
        # err1_prev = err1
        err_prev = err

        self.cmd_vel_pub.publish(self.twist)
        output_img = self.bridge.cv2_to_imgmsg(line_img[0:h, 0:w])
        self.image_pub.publish(output_img)
        # ==================================================================== #

    ## : def image_callback(self, msg) 함수에서 line의 중심점을 찾기 위해 사용
    # 이 함수는 모든 픽셀값에 대한 평균으로 정확한 평균 위치를 찾아주진 않지만 
    # 중위값을 계산함으로써 위 함수보다 빠르다. 또 노이즈 제거 효과도 있어서 calc_center_point_valavr보다 오히려 성능이 더 좋음.
    def calc_center_point_idxavr(self, bframe, threshold): #-> int
        
        # 이미지의 가로 인덱스별 픽셀 갯수 ndarray반환 : bframe_hist는 1차 ndarray 타입
        bframe_hist = numpy.sum(bframe,axis=0) 
        
        # 픽셀갯수가 threshold값을 넘는 ndarray의 index 반환, 1차 ndarray를 인수로 받았기 때문에 출력은 (1차 ndarray,) 튜플로 반환됨, 그래서 [0]을 사용
        pixel_idx_list = (numpy.where(bframe_hist > threshold))[0] 

        if len(pixel_idx_list) > 4 :
            # 즉, [가로 인덱스마다 세로로 합친 픽셀 갯수가 threshold값을 넘어가는 가로 인덱스값들](=[pixel_idx_list])의 평균을 반환
            self.roi_center_pix = int(numpy.average(pixel_idx_list))
            
# END CONTROL #


rospy.init_node("follower")
follower = Follower()
rospy.spin()
# END ALL
