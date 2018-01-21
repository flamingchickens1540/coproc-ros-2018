#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image


cameraMatrix = np.array([[660.05256, 0.0, 330.85059],
                         [0.0, 662.95370, 242.03401],
                         [0.0, 0.0, 1.0]], dtype="double")

distCoeffs = np.array((0.02752, -0.238537, 0.001611, -0.0073284, 0))

def main():
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
    camera.set(cv2.CAP_PROP_SATURATION, 255)
    camera.set(cv2.CAP_PROP_EXPOSURE, 0)

    # camera = cv2.VideoCapture(0)
    # camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    # camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
    # camera.set(cv2.CAP_PROP_SATURATION, 255)
    # camera.set(cv2.CAP_PROP_EXPOSURE, -50)

    pipeline = grip.ContoursPipeline()

    cv2.destroyAllWindows()





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        camera.release()
        pass

        #!/usr/bin/env python

import rospy
import tf
from networktables import NetworkTables

from geometry_msgs.msg import *

left_vel = 0
right_vel = 0

def main():
        twist_pub = rospy.Publisher("~twist", TwistWithCovarianceStamped, queue_size=10)

        NetworkTables.initialize(server="10.15.40.100")
        robotTable = NetworkTables.getTable("rosTable")

        robotTable.addTableListener(left_vel_cb, key="LeftVelocity")
        robotTable.addTableListener(right_vel_cb, key="RightVelocity")

        #TODO make this a param
        wheel_separation = 0.622
        wheel_circ = 0.562
        rospy.loginfo("Hello")

        rospy.spin()

        # image_sub = rospy.Subscriber("head_camera/image_raw", Image, self.image_cb)
def left_vel_cb(source, key, value, isNew):
    left_vel = value*wheel_circ/60
    publishTwist()

def right_vel_cb(source, key, value, isNew):

    right_vel = value*wheel_circ/60
    rospy.loginfo(value)
    publishTwist()

# def publishTwist():
#     linearVel = (right_vel+left_vel)*0.5
#     angularVel = (right_vel-left_vel)/wheel_separation
#     header = Header(stamp=rospy.Time.now(), frame_id='map')
#     covar = [
#             0.001, 0, 0, 0, 0, 0,
#             0, 0.001, 0, 0, 0, 0,
#             0, 0, 0.001, 0, 0, 0,
#             0, 0, 0, 0.001, 0, 0,
#             0, 0, 0, 0, 0.001, 0,
#             0, 0, 0, 0, 0, 0.003]
#     twcs = TwistWithCovarianceStamped(header, TwistWithCovariance(Twist(Vector3(linearVel, 0, 0), tf.transformations.quaternion_from_euler(0, 0, angularVel))), covar)
#     twist_pub.Publish(twcs)


if __name__ == '__main__':
    rospy.init_node('diff_drive_to_twist', anonymous=False)
    try:
        main()
    except rospy.ROSInterruptException:
        pass