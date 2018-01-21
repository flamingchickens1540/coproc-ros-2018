#!/usr/bin/env python

import grip
import rospy
import cv2
import numpy as np
import tf
import sys

from tf import transformations
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
from std_msgs.msg import Header
from sensor_msgs.msg import Image

class PolyPose:
    def __init__(self):
        # TODO: Use parameters

        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=10)
        self.point_pub = rospy.Publisher("~point", PointStamped, queue_size=10)
        self.image_pub = rospy.Publisher("~image_debug", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("head_camera/image_raw", Image, self.image_cb)

        # TODO: Read camera calibration parameters from YAML file
        self.cameraMatrix = np.array([[1179.018046, 0.0, 643.553610],
                        [0.0, 1177.297209, 319.386619],
                        [0.0, 0.0, 1.0]], dtype="double")

        self.distCoeffs = np.array((-0.009014, -0.030282, -0.000673, 0.001892, 0))

        # self.cameraMatrix = np.array([[660.05256, 0.0, 330.85059],
        #                 [0.0, 662.95370, 242.03401],
        #                 [0.0, 0.0, 1.0]], dtype="double")

        # self.distCoeffs = np.array((0.02752, -0.238537, 0.001611, -0.0073284, 0))

        # self.cameraMatrix = np.array([[1124.3044, 0.0, 635.909862],
        #                 [0.0, 1143.866863, 334.549318],
        #                 [0.0, 0.0, 1.0]], dtype="double")

        # self.distCoeffs = np.array((0.030202, -0.218237, 0.00, 0.00, 0))


        # TODO: Read object points from input YAML file
        self.objectPoints = np.array([(2.5375, 4.0155, 0.0),
                                (-2.5375, 4.0155, 0.0),
                                (2.5375, -4.0155, 0.0),
                                (-2.5375, -4.0155, 0.0)])

        self.pipeline = grip.ContoursPipeline()

    def image_cb(self, input_image):
        # Convert ROS image message to an opencv image
        try:
            outputImage = self.bridge.imgmsg_to_cv2(input_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv2 to imgmsg")
            rospy.logerr(e)

        # Process the image using the GRIP pipeline
        self.pipeline.process(outputImage)

        # Filter contours to polygons with four edges
        filtered = self.filterPoly(self.pipeline.filter_contours_output, 4, 20)
        cv2.drawContours(outputImage, filtered, -1, (0, 0, 255), 2)

        if len(filtered) > 0:
            for contour in filtered:
                pointList = sorted(list(contour), key=lambda point: point[0][1])

                if pointList[3][0][0] > pointList[2][0][0]:
                    pointList.insert(2, pointList.pop(3))

                if pointList[1][0][0] > pointList[0][0][0]:
                    pointList.insert(0, pointList.pop(1))

                test, rvec, tvec = cv2.solvePnP(self.objectPoints,
                                                np.ascontiguousarray(pointList, dtype="double"),
                                                self.cameraMatrix,
                                                self.distCoeffs)

                # newPose=Pose(Point(tvec[0][0], tvec[1][0], tvec[2][0]), Quaternion(*quat))
                # self.pose_pub.publish(PoseStamped(Header(stamp=rospy.Time.now(), frame_id='map'), newPose))
                self.pose_pub.publish(PoseStamped(
                    Header(stamp=rospy.Time.now(), frame_id='map'), 
                    Pose(Point(tvec[0][0], tvec[1][0], tvec[2][0]), 
                    Quaternion(*tf.transformations.quaternion_from_euler(rvec[0][0], rvec[1][0], rvec[2][0])))))
                self.point_pub.publish(PointStamped(Header(stamp=rospy.Time.now(), frame_id='map'), Point(tvec[0][0], tvec[1][0], tvec[2][0])))
                outputImage = self.drawPose(outputImage, rvec, tvec, self.cameraMatrix, self.distCoeffs)
            
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(outputImage, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)
        
        

    def filterPoly(self, contours, vertices, epsilon):
        output = []
        for ii in range(len(contours)):
            polyContour = cv2.approxPolyDP(contours[ii], epsilon, True)
            if len(polyContour) == vertices:
                output.append(polyContour)
        return output


    def drawPose(self, image, rvec, tvec, cameraMatrix, distCoeffs):
        anglePoints3D = np.array([(0, 0, 0),
                                (2, 0, 0),
                                (0, 2, 0),
                                (0, 0, 2)], dtype="double")

        anglePoints2D = cv2.projectPoints(anglePoints3D, rvec, tvec, cameraMatrix, distCoeffs)[0]

        pointsList = [tuple(anglePoints2D[0][0].astype(int)),
                    tuple(anglePoints2D[1][0].astype(int)),
                    tuple(anglePoints2D[2][0].astype(int)),
                    tuple(anglePoints2D[3][0].astype(int))]
        try:
            cv2.line(image, pointsList[0], pointsList[1], (0, 0, 255), 2)
            cv2.line(image, pointsList[0], pointsList[2], (255, 0, 0), 2)
            cv2.line(image, pointsList[0], pointsList[3], (0, 100, 0), 2)
        except OverflowError:
            pass

        return image

if __name__ == '__main__':
    rospy.init_node('rect_test', anonymous=False)
    pp = PolyPose()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass