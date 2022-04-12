#!/usr/bin/env python2.7
from cmath import isnan
from tkinter import N
from turtle import color
import rospy
from sensor_msgs.msg import Image as im
import numpy as np
from PIL import Image, ImageFilter
import PIL
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError

print ("Hello!")
rospy.init_node('opencv_example', anonymous=True)
rospy.loginfo("Hello ROS!")
bridge = CvBridge()

def RunAvg(image,aWeight,bg):

    if bg is None:

        bg = image.copy().astype("float")

        return bg

    return cv2.accumulateWeighted(image,bg,aWeight)

class FeedImage():

    def __init__(self,image_shape,image_type = 'cv2',return_type = 'canny'):
        self.shape = image_shape
        self.type = image_type
        self.return_type = return_type

    def parse(self,img,blur_kernel=(7,7)):
        if self.type == 'cv2':
            image = cv2.resize(src = img, dsize = self.shape)
        if self.type == 'PIL':
            image = np.array(img)
            image = cv2.resize(image)
        blur = cv2.GaussianBlur(src = image, ksize = blur_kernel,sigmaX=1)
        return blur

    def enhance(self,img,enhancements = []):
        if not enhancements:
            img = PIL.Image.fromarray(img)
            image = img.filter(ImageFilter.UnsharpMask(radius = 3, percent = 200, threshold = 5))
            return np.array(image)

        return np.array(image)

    def segment(self,img,K):
        twoDimage = img.reshape((-1,3))
        twoDimage = np.float32(twoDimage)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        attempts=10
        _,label,center= cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
        _,label,center= cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)
        center = np.uint8(center)
        res = center[label.flatten()]
        result_image = res.reshape((img.shape))

        return result_image

    def contour(self,img,canny_thresh1 = 100,canny_thresh2 = 200):
        img = PIL.Image.fromarray(img)
        img = img.filter(ImageFilter.UnsharpMask(radius = 3, percent = 200, threshold = 5))
        img = np.array(img)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img_blur = cv2.GaussianBlur(img_gray, (3,3), 0)
        sobelx = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5)
        sobely = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) 
        sobelxy = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)
        edges = cv2.Canny(image=img_gray, threshold1=canny_thresh1, threshold2=canny_thresh2)
        if self.return_type == 'canny':
            return edges

    def highlights(self,img):

        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)

        twoDimage = img.reshape((-1,3))

        twoDimage = np.float32(twoDimage)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

        K = 2

        attempts=10

        ret,label,center=cv2.kmeans(twoDimage,K,None,criteria,attempts,cv2.KMEANS_PP_CENTERS)

        center = np.uint8(center)

        res = center[label.flatten()]

        result_image = res.reshape((img.shape))

        # cv2.waitKey(0)

        return result_image

    def get_img(self,img):

        parsed = self.parse(img,(3,3))

        high = self.highlights(parsed)

        return_image = high

        return return_image


def centerline(result_image):

    process = FeedImage(image_shape=(640,480))

    result_image = process.get_img(result_image)

    gr_res = cv2.cvtColor(result_image,cv2.COLOR_BGR2GRAY)

    # cv2.imshow('kk',gr_res)

    # cv2.waitKey(0)

    (a1,a2),(con1,con2) = np.unique(gr_res,return_counts=True)

    if con1>0.6*(con2+con2):

        a2 = a1

    ver, hor = gr_res.shape

    edge_points = np.argwhere(list(result_image[:,:,1] == a2))

    centerline_coordinates = []

    for i in range(0,ver,10):
        
        cen = (np.mean((edge_points[edge_points[:,0]==i]),axis=0).astype(int))

        centerline_coordinates.append(cen[::-1])

        result_image = cv2.circle(result_image,center=tuple(cen[::-1]),color=(0,0,255),thickness=-1,radius=30)

    return result_image, centerline_coordinates


def show_image(img,str):
    cv2.imshow(str, img)
    cv2.waitKey(3)

def image_callback(img_msg):
    
    global Kp, Ki, kd, integral, derivative, last_error, Kp_ang, Ki_ang, kd_ang, integral_ang, derivative_ang, last_ang, was_line, line_side, battery, line_back, landed, takeoffed, fly_time, start, stop, velocity
    
    global img_pub, pub_vel, while_loop_counts, processed_image_old

    # rospy.loginfo(img_msg.header)

    try:
    
        cv_image = bridge.imgmsg_to_cv2(img_msg,"32FC1")
    
    except CvBridgeError as e:
    
        rospy.logerr("CvBridge Error: {0}".format(e))

    cv_image = np.where(np.isnan(cv_image), 0, cv_image)

    roi = cv_image[120:360, 160:480]

    # avg_image = cv_image.copy()
    # if while_loop_counts%10 != 0:
    #     avg_image = RunAvg(avg_image, 0.5, avg_image)

    # else:

    #     cv_image = avg_image
    #     print("avg frame")



    down_vel = 0

    ma = np.max(roi)

    # print(ma)

    if (ma>15):

        down_vel = -0.2

        # print("moved down")
    
    elif ma<12:
    
        down_vel = 0.2

    cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
    
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    
    cv_image_norm = np.uint8(cv_image_norm * 255)
    
    cv_new = cv2.cvtColor(cv_image_norm, cv2.COLOR_GRAY2BGR)


    if while_loop_counts%20 == 0:

        processed_image, coord = centerline(cv_new[0:200,:])
        processed_image_old = processed_image
    
    
    processed_image = processed_image_old 
    
    while_loop_counts += 1

    image_show = processed_image.copy()

    processed_image = cv2.cvtColor(processed_image[120:360],cv2.COLOR_BGR2HSV)

    hsv = cv2.cvtColor(processed_image, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    lower_red = np.array([0,50,50]) #example value
    
    upper_red = np.array([0,255,255])
    
    mask = cv2.inRange(processed_image, lower_red, upper_red)
    
    cv2.imshow('mask',mask)
    
    # kernel = np.ones((3, 3), np.uint8)
    
    # mask = cv2.erode(mask, kernel, iterations=5)
    
    # mask = cv2.dilate(mask, kernel, iterations=9)
    
    # cv2.imshow('pro',mask)
    
    # cv2.waitKey(0)
    _, contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blk.sort(key=cv2.minAreaRect)

    if len(contours_blk) > 0 and cv2.contourArea(contours_blk[0]) > 5000:
        was_line = 1
        blackbox = cv2.minAreaRect(contours_blk[0])
        (x_min, y_min), (w_min, h_min), angle = blackbox
        if angle < -45:
            angle = 90 + angle
        if w_min < h_min and angle > 0:
            angle = (90 - angle) * -1
        if w_min > h_min and angle < 0:
            angle = 90 + angle

        setpoint = cv_image.shape[1] / 2
        error = int(x_min - setpoint)
        # error.append(error)
        # angle.append(angle)
        normal_error = float(error) / setpoint

        if error > 0:
            line_side = 1  # line in right
        elif error <= 0:
            line_side = -1  # line in left

        integral = float(integral + normal_error)
        derivative = normal_error - last_error
        last_error = normal_error


        error_corr = -1 * (Kp * normal_error + Ki * integral + kd * derivative)  # PID controler
        # print("error_corr:  ", error_corr, "\nP", normal_error * self.Kp, "\nI", self.integral* self.Ki, "\nD", self.kd * self.derivative)

        angle = int(angle)
        integral_ang = float(integral_ang + angle)
        derivative_ang = angle - last_ang
        last_ang = angle
        ang_corr = -1 * (Kp_ang * angle + Ki_ang * integral_ang + kd_ang * derivative_ang)  # PID controler

        # box = cv2.boxPoints(blackbox)
        # box = np.int0(box)
        # cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 3)
        # cv2.putText(cv_image, "Angle: " + str(angle), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
        #             cv2.LINE_AA)

        # cv2.putText(cv_image, "Error: " + str(error), (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
        #             cv2.LINE_AA)
        # cv2.line(cv_image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)


        twist = Twist()
        twist.linear.x = velocity
        # twist.linear.y = 0
        twist.linear.y = 0
        twist.linear.z = down_vel
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = ang_corr*0.4 + error_corr*0.6

        # print(error_corr, ang_corr)
        pub_vel.publish(twist)
            

    if len(contours_blk) == 0 and was_line == 1 and line_back == 1:
        twist = Twist()
        if line_side == 1:  # line at the right
            twist.linear.y = -0.05
            pub_vel.publish(twist)
        if line_side == -1:  # line at the left
            twist.linear.y = 0.05
            pub_vel.publish(twist)

    # print('s',cv_image.shape)
    image_message = bridge.cv2_to_imgmsg(image_show,  encoding="passthrough")
    img_pub.publish(image_message)


Kp = 0.112                 # Ku=0.14 T=6. PID: p=0.084,i=0.028,d=0.063. PD: p=0.112, d=0.084/1. P: p=0.07
Ki = 0
kd = 1
integral = 0
derivative = 0
last_error = 0
Kp_ang = 0.01             # Ku=0.04 T=2. PID: p=0.024,i=0.024,d=0.006. PD: p=0.032, d=0.008. P: p=0.02/0.01
Ki_ang = 0
kd_ang = 0
integral_ang = 0
derivative_ang = 0
last_ang = 0
was_line = 0
line_side = 0
battery = 0
line_back = 1
landed = 0
# error = [], angle =[]
takeoffed = 0
fly_time = 0.0
start = 0.0
stop = 0.0
velocity = 0.6
while_loop_counts = 0
sub_image = 0
img_pub = 0
pub_vel = 0
twist = 0
processed_image_old = float

def main():
    global sub_image, img_pub, pub_vel, twist
    sub_image = rospy.Subscriber("/depth_camera/depth/image_raw", im, image_callback)
    img_pub = rospy.Publisher("/image_topic_2",im, queue_size = 10)
    pub_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped",Twist, queue_size=1)
    twist = Twist()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    main()