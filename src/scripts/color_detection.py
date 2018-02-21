#!/usr/bin/env python
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
from geometry_msgs.msg import Point

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # topic where we publish
        self.image_pub = rospy.Publisher("/color_detection/image",
            Image, queue_size=10)
        self.bridge = CvBridge()
        #topic where the coordinates go
        self.cam_pose_pub = rospy.Publisher("/color_detection/cam_point", Point, queue_size=1)
        self.cam_pose = Point()
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/uav_camera/image_raw_down", Image, self.callback,  queue_size=1)
        if VERBOSE :
            print "subscribed to /uav_camera/image_raw_down"


    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        ###Set boundarys###
        lower_range = np.array([0, 255, 255], dtype=np.uint8) #The object color is 255,153,0
        upper_range = np.array([0, 0, 255], dtype=np.uint8)

        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        yellow_lr = np.array([20, 100, 100], dtype=np.uint8)
        yellow_ur = np.array([30, 255, 255], dtype=np.uint8)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        image, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        center = Point()
        #print(contours)
        # only proceed if at least one contour was found
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)

           # ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            #center.x = x
            #center.y = y
            center.x = float(M["m10"] / M["m00"])
            center.y = float(M["m01"] / M["m00"])
            self.find_relative_pose(center.x, center.y, w, h)
            #self.c_coord_pub.publish(center)
            cv2.rectangle(cv_image, (x, y),(x+w, y+h), (0, 255, 255), 2)
        else:
            self.cam_pose = Point()
            self.cam_pose.x = float("inf")
            self.cam_pose.y = float("inf")
            self.cam_pose.z = float("inf")
            self.cam_pose_pub.publish(self.cam_pose)
        #cv2.imshow('cv_img', cv_image)
        #cv2.waitKey(2)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        # #### Feature detectors using CV2 ####
        # # "","Grid","Pyramid" +
        # # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        # method = "GridFAST"
        # feat_det = cv2.ORB_create()
        # time1 = time.time()
        #
        # # convert np image to grayscale
        # featPoints = feat_det.detect(
        #     cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        # time2 = time.time()
        # if VERBOSE :
        #     print '%s detector found: %s points in: %s sec.'%(method,
        #         len(featPoints),time2-time1)
        #
        # for featpoint in featPoints:
        #     x,y = featpoint.pt
        #     cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
        #
        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)
        #
        # #### Create CompressedIamge ####
        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # # Publish new image
        # self.image_pub.publish(msg)
        #
        # #self.subscriber.unregister()
    def find_relative_pose(self, x,y,w,h):
        quad_3d = np.float32([[-0.25, -0.25, 0], [0.25, -0.25, 0], [0.25, 0.25, 0], [-0.25, 0.25, 0]])
        #nere vanstra hornet
        quad = np.float32([[x - w / 2, y - h / 2], [x - w / 2, y + h / 2], [x + w / 2, y + h / 2], [x + w / 2, y - h / 2]])
        #uppe vanstra hornet
        #quad=np.float32([[x-w/2, y+h/2], [x+w/2, y+h/2], [x+w/2, y-h/2], [x-w/2, y-h/2]])
        # uppe hogra hornet
        #quad=np.float32([[x+w/2, y+h/2], [x+w/2, y-h/2], [x-w/2, y-h/2], [x-w/2, y+h/2]])
        # nere hogra hornet
        #quad=np.float32([[x+w/2, y-h/2], [x-w/2, y-h/2], [x-w/2, y+h/2], [x+w/2, y+h/2]])
        K = np.float64([[1004.603218, 0, 640.5],
                        [0, 1004.603218, 360.5],
                        [0.0, 0.0, 1.0]])
        dist_coef = np.zeros(4)
        _ret, rvec, tvec = cv2.solvePnP(quad_3d, quad, K, dist_coef)
        rmat = cv2.Rodrigues(rvec)[0]
        cameraRotVector = cv2.Rodrigues(np.matrix(rmat).T)
        cameraTranslatevector = -np.matrix(rmat).T * np.matrix(tvec)
        self.cam_pose.x = cameraTranslatevector.item((0,0))
        self.cam_pose.y = cameraTranslatevector.item((1,0))
        self.cam_pose.z = cameraTranslatevector.item((2,0))
        self.cam_pose_pub.publish(self.cam_pose)
        #camera_position = -np.matrix(rmat).T * np.matrix(tvec)
        #print(camera_position)

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('color_detection', anonymous=True)
    rospy.Rate(20)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image color detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
