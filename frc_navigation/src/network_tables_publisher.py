#!/usr/bin/env python

import rospy
import tf
from tf import *
from networktables import NetworkTables

from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *

class NetworkTablesPublisher:
    def __init__(self):
        self.wheel_separation = 0.622*1.87
        self.wheel_circ = 0.562

        self.twist_pub = rospy.Publisher("~twist", TwistWithCovarianceStamped, queue_size=10)
        self.imu_pub = rospy.Publisher("~imu", Imu, queue_size=10)
        
        self.twist_sub = rospy.Subscriber("robot_pub/cmd_vel", Twist, self.twist_cb)

        NetworkTables.initialize(server="10.15.40.100")
        self.robotTable = NetworkTables.getTable("rosTable")

        self.robotTable.addTableListener(self.update_cb, key="RobotTime")

        self.imu_yaw = self.robotTable.getAutoUpdateValue("IMU_Yaw", 0)
        self.left_vel = self.robotTable.getAutoUpdateValue("left_vel", 0)
        self.right_vel = self.robotTable.getAutoUpdateValue("right_vel", 0)

        self.robotTable.putNumber('left_cmd', 0)
        self.robotTable.putNumber('right_cmd', 0)

        rospy.spin()



    def twist_cb(self, twist):
        self.robotTable.putNumber('left_cmd', (twist.linear.x*1.6 - twist.angular.z*1.9 * self.wheel_separation / 2.0)/self.wheel_circ)
        self.robotTable.putNumber('right_cmd', (twist.linear.x*1.6 + twist.angular.z*1.9 * self.wheel_separation / 2.0)/self.wheel_circ)
        

    def update_cb(self, source, key, value, isNew):
        #-1 IS WITH
        #1 IS AWAY
        timestamp = rospy.Time(value/1000)-rospy.Duration(0.07)
        self.imu_yaw_cb(timestamp)
        self.enc_vel_cb(timestamp)

    def enc_vel_cb(self, timestamp):
        right_vel = self.right_vel.value*self.wheel_circ/60
        left_vel = self.left_vel.value*self.wheel_circ/60
        
        linearVel = (right_vel+left_vel)*0.5*0.6
        angularVel = (right_vel-left_vel)/self.wheel_separation
        header = Header(stamp=timestamp, frame_id='base_link')
        covar = [
                0.02, 0, 0, 0, 0, 0,
                0, 0.02, 0, 0, 0, 0,
                0, 0, 0.02, 0, 0, 0,
                0, 0, 0, 0.2, 0, 0,
                0, 0, 0, 0, 0.2, 0,
                0, 0, 0, 0, 0, 0.2]
        twcs = TwistWithCovarianceStamped(header, TwistWithCovariance(Twist(Vector3(linearVel, 0, 0), Vector3(0, 0, angularVel)), covar))
        self.twist_pub.publish(twcs) 

    def imu_yaw_cb(self, timestamp):
        covar = [
                    0.01, 0, 0,
                    0, 0.01, 0,
                    0, 0, 0.01]
        imu = Imu(Header(stamp=timestamp, frame_id='base_link'), Quaternion(*tf.transformations.quaternion_from_euler(0,0,self.imu_yaw.value/-180*3.14)), covar, Vector3(0,0,0), covar, Vector3(0,0,0), covar)
        self.imu_pub.publish(imu) 

if __name__ == '__main__':
    rospy.init_node('network_tables_pub', anonymous=False)
    thing = NetworkTablesPublisher()