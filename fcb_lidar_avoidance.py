# 개발 초기 코드

#!/usr/bin/env python
# BEGIN ALL
import rospy
import cv2
import cv_bridge
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from traffic_light_classifier.msg import traffic_light

global perr, ptime, serr, dt, move, ray_angle
perr = 0
ptime = 0
serr = 0
dt = 0
move = False
angle_step_deg = 20


class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/usb_cam/image_raw',	Image, self.image_callback)
		self.lidar_sub = rospy.Subscriber('/scan_raw', LaserScan, self.lidar_callback)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.image_pub = rospy.Publisher('/lane_image', Image, queue_size=1)
		self.twist = Twist()
		self.ray_angle = [x for x in range(angle_step_deg, 180, angle_step_deg)]
		self.dists = None

		self.cmd_vel_pub.publish(self.twist)

	def lidar_callback(self, msg):

		# static offset
		angles = [x for x in range(-80, 81, 5)]
		self.dists = [msg.ranges[x*2] for x in angles]


	def get_obstacle_threshold(self):
		if self.dists == None:
			return 0

		# dynamic offset
		# lateral_dists = [dist * numpy.cos(numpy.deg2rad(theta)) for dist, theta in zip(self.dists, self.ray_angle)]

		# static offset
		lateral_count_left = 0
		lateral_count_right = 0
		
		for i in range(-80, 81, 5):
			d = self.dists[i]
			if i < 0 and d < 0.15:
				lateral_count_left += 1
			elif i > 0 and d < 0.15:
				lateral_count_right += 1

		if lateral_count_left > lateral_count_right and lateral_count_left >= 30:
			print("lateral_left_cnt :{}".format(lateral_count_left))
			return -75
		
		elif lateral_count_right > lateral_count_left and lateral_count_right >= 30:
			print("lateral_right_cnt :{}".format(lateral_count_right))
			return 75

			# dynamic offset		# return sum(lateral_dists)





	def image_callback(self, msg):
		global perr, ptime, serr, dt
		image0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


		# transformation
		img = cv2.resize(image0, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_CUBIC)
		

        #print img.shape
		rows, cols, ch = img.shape
		#print(rows,cols,ch)
		pts1 = numpy.float32([[65,120], [49,144], [140,120], [161,144]])
		pts2 = numpy.float32([[0, 0], [0, 300], [300, 0], [300, 300]])
		

		M = cv2.getPerspectiveTransform(pts1, pts2)
		img_size = (img.shape[1], img.shape[0])
		image = cv2.warpPerspective(img, M, (300, 300))  # img_size

		
		#cv2.imwrite('save_img.jpg',img)
		
		#cv2.imshow('img', img)
                #cv2.imshow('image' ,image)
		#cv2.waitKey(0)
		#cv2.destroyAllWindows()
		#result = cv2.imwrite('image_test2.jpg',img)


		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


		lower_red = numpy.array([156,  43,  46])
		upper_red = numpy.array([180, 255, 255])
		lower_red1 = numpy.array([0, 43, 46])
		upper_red1 = numpy.array([10, 255, 255])

		maskr = cv2.inRange(hsv, lower_red, upper_red)
		maskr1 = cv2.inRange(hsv,lower_red1, upper_red1)

		maskrr = cv2.bitwise_or(maskr,maskr1)

		# lower_white = numpy.array([0,0,221])
		# upper_white = numpy.array([180,30,225])
		# lower_white = numpy.array([0,0,210])
		# upper_white = numpy.array([200,255,255])
		sensitivity = 30
		lower_white = numpy.array([0, 0, 255-sensitivity])
		upper_white = numpy.array([255, sensitivity, 255])

		# lower_black = numpy.array([0,0,4])
		# upper_black = numpy.array([180,255,140])
		lower_black = numpy.array([0, 0, 10])
		upper_black = numpy.array([180, 255, 100])

		# threshold to get only white
		maskw = cv2.inRange(hsv, lower_white, upper_white)
		maskb = cv2.inRange(hsv, lower_black, upper_black)
		# maskb = cv2.inRange(gray, 0, 120)

		# masky = cv2.inRange(hsv, lower_yellow, upper_yellow)
		# remove pixels not in this range
		#maskyw = cv2.bitwise_or(maskw, masky)
		#maskyb = cv2.bitwise_or(maskb, masky)

		#rgb_yb = cv2.bitwise_and(image, image, mask = mask_yb).astype(numpy.uinet8)
		# rgb_yb = cv2.bitwise_and(image, image, mask = maskb).astype(numpy.uint8)
		rgb_yb = cv2.bitwise_and(image, image, mask=maskrr).astype(numpy.uint8)
		rgb_yb = cv2.cvtColor(rgb_yb, cv2.COLOR_RGB2GRAY)
		

		# filter mask
		kernel = numpy.ones((7, 7), numpy.uint8)
		opening = cv2.morphologyEx(rgb_yb, cv2.MORPH_OPEN, kernel)
		rgb_yb2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

		#_, rgb_yb2 = cv2.threshold(rgb_yb2, 30, 255, cv2.THRESH_BINARY) # Jooseok
		
	
		#out_img = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel) # Jooseok

		# ROI
		out_img = rgb_yb2.copy()
		h, w = out_img.shape #rows=144 cols=192
		
		#print(h,w)
		search_top = int(1*cols/4+20) #
		search_bot = int(3*cols/4+20) #
		search_mid = int(rows/2)
		out_img[0:search_top, 0:w] = 0
		out_img[search_bot:h, 0:w] = 0

		#_, contour, _ = cv2.findContours(out_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		M = cv2.moments(out_img)
		c_time = rospy.Time.now()
		if M['m00'] > 0:
			cxm = int(M['m10']/M['m00'])
			cym = int(M['m01']/M['m00'])
			print('cxm {0}'.format(cxm))
			print('cym {0}'.format(cym))

			# cx = cxm - 110 #120#143 #CW
			#cx = cxm - 150
			offset = self.get_obstacle_threshold()
			#print("offset: ", offset)
			cx = cxm - offset

			cv2.circle(out_img, (cxm, cym), 20, (255, 0, 0), -1)
			cv2.circle(out_img, (cx, cym), 20, (255, 0, 0), 2)			
			

			# BEGIN CONTROL
			err = cx - 4*w/8
		#   K_p = 0.63
			ang_user= 0.1
			K_p = 0.3 #default==0.4
			K_i = 0.005 #default==0.0005
			K_d = 0.4 #default==0.005
			p_error = 0.0
			i_error = 0.0
			d_error = 0.0
			
			d_error = err-p_error
			p_error = err
			i_error = i_error + err

		#   self.twist.linear.x = K_p
			dt = rospy.get_time() - ptime
		#   self.twist.angular.z = (-float(err) / 100)*2.5 + ((err - perr)/(rospy.get_time() - ptime))*1/50/100 #+ (serr*dt)*1/20/100 #1 is best, starting 3 unstable
		#   ang_z = err*0.0028
			# + (serr*dt)*1/20/100 #1 is best, starting 3 unstable
			if cxm-cym < -100 or cxm-cym > 100:
				ang_user = 4
			ang_z = K_p*p_error + K_i*i_error + K_d*d_error
			ang_z = min(ang_user, max(-ang_user, ang_z)) #default==0.1
		# 0.143
			lin_x = ang_z
			if lin_x < 0:
				lin_x = -(lin_x)
			lin_x = K_p * (1-lin_x)

			#self.twist.linear.x = lin_x
			self.twist.linear.x = 0.4
			self.twist.angular.z = -ang_z

		#   print(cx, err*0.02, ang_z)
			# print("cx:{}, err:{:.4f}, ang_z:{:4f}, lin_x:{:4f}".format(
			#	cx, err*0.0015, ang_z, lin_x))
			serr = err + serr
			perr = err
			ptime = rospy.get_time()

		else:
			self.twist.linear.x = 0.1
			#self.twist.angular.z = -0.3
			err = 0

		self.cmd_vel_pub.publish(self.twist)
		#?��기서 out_img?��미�??�? rgb?�� hsv�? 바꿔보기 !
		output_img = self.bridge.cv2_to_imgmsg(out_img)
		self.image_pub.publish(output_img)
		#?��차피 ?��기있?��!!

		# END CONTROL

		#cv2.imshow("win3", rgb_yw2)
		# cv2.waitKey(3)


rospy.init_node('follower')
global check
check = 0

def traffic_check_func(msg):
	check = msg

traffic_check = rospy.Subscriber('light_color',traffic_light, traffic_check_func)

if(check == 1):
	follower = Follower()

rospy.spin()
# END ALL
