#!/usr/bin/env python
import cv2
import cv2.aruco as aruco
import numpy as np
from time import sleep
import rospy
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped as Pose
import tf
#camera calibration parameters
global cameraMatrix
global distCoeffs

odom_pub = rospy.Publisher('/camera_pose', Pose, queue_size=100)

# tf listener object used to transform pose data in camera frame to base_link frame
tf_listen = tf.TransformListener()

#obtain the predefined dictionary and generate the aruco Detector parameters
aruco_dict =  aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()
bridge = cv_bridge.CvBridge()
def image_callback(image):
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    pos_msg = Pose()

    #read and convert to grayscale for analysis
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #
    #obtain aruco marker corners and ids
    #generate the pose estimation for a single aruco marker with the defined corners and camera parameter
    #gain rotational and translational vectors
    corners, ids, _= aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    rvecs, tvecs, pts  = aruco.estimatePoseSingleMarkers(corners, 0.07, cameraMatrix, distCoeffs)
    if tvecs is not None:
        pos_msg.pose.position.x = tvecs[0][0][2]
        pos_msg.pose.position.y = tvecs[0][0][0]
        pos_msg.pose.position.z = tvecs[0][0][1]

    if rvecs is not None:
        quaternion = tf.transformations.quaternion_from_euler(rvecs[0][0][1], rvecs[0][0][2], rvecs[0][0][0])
        pos_msg.pose.orientation.x = quaternion[0]
        pos_msg.pose.orientation.y = quaternion[1]
        pos_msg.pose.orientation.z = quaternion[2]
        pos_msg.pose.orientation.w = quaternion[3]


    # transform data from camera to base_link frame

    # first, block thread until transform is available
    try:
        corr_time = tf_listen.getLatestCommonTime('camera', 'base_link')
        rospy.loginfo(corr_time)
        pos_msg.header.stamp = corr_time
        pos_msg.header.frame_id = 'camera'
        #rospy.loginfo(pos_msg)
        
        try:
            tf_listen.waitForTransform('camera', 'base_link', corr_time, rospy.Duration(4.0))
            try:
                rospy.loginfo('now Im here')
                pos_msg_odom = tf_listen.transformPose('base_link', pos_msg)
                if tvecs is not None and rvecs is not None:
                    odom_pub.publish(pos_msg_odom)
                    rospy.loginfo("published")
            except Exception as error:
                rospy.loginfo('now Imdfdfdfdf here')
                rospy.loginfo(error)


        except Exception as error:
            rospy.loginfo('now asdfasdfIm here')
            rospy.loginfo(error)
    except Exception as error:
        rospy.loginfo('now Isdafasdfsdafm here')
        rospy.loginfo(error)






def camera_info_callback(CameraInfo):
    global cameraMatrix
    global distCoeffs
    bfx = 1041.2
    bfy = 1040.5
    bcx = 639.1
    bcy = 349.1
    br1 = 0.0881
    br2 = -0.2258
    br3 = 0.2579
    cameraMatrix = np.matrix([ [bfx, 0, bcx], [0, bfy, bcy], [0, 0, 1] ])
    distCoeffs = np.matrix([ br1, br2, 0, 0, br3])
    distCoeffs = CameraInfo.D
    #cameraMatrix = CameraInfo.K

def main():

    rospy.init_node('aruco', anonymous=True)
    rospy.Subscriber('/cam1/image', Image, image_callback)
    rospy.Subscriber('/cam1/camera_info', CameraInfo, camera_info_callback)

    rospy.spin()


if __name__ == '__main__':
    main()



