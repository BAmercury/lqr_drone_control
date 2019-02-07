import sys
import cv2
import cv2.aruco as aruco
#help (cv2.aruco)

#Generate Aruco marker from predefined dictionary
aruco_dict =  aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
# markers (23), 100x100 pixels, 1 bit default border - Can be altered as an additioanl parameter
img = aruco.drawMarker(aruco_dict, 23, 100)
#generate the marker and store in predefined path
cv2.imwrite("aruco_marker.jpg", img)
