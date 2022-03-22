import rospy
import math
import cv2
import numpy as np
from cv_bridge import CvBridge
from interiit21.srv import*
from mavros_msgs.srv import SetMode
from prius_msgs.msg import Control
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


class drone_control:
    def move_to_destination(self,x,y,z,yaw,frame_id='fcu_horiz'):
        rospy.wait_for_service('set_position')
        try:
            set_position = rospy.ServiceProxy('set_position', SetPosition)
            resp1 = set_position(x = x, y = y ,z = z, yaw = yaw, frame_id=frame_id)
            rospy.loginfo(resp1.success)
            return resp1.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def land(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            resp1 = set_mode(custom_mode='LAND')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

class car_control:
    def __init__(self,accel_gain,brake_gain,steer_gain) :
        # rospy.init_node('car_control')
        self.pub = rospy.Publisher('/prius', Control, queue_size=10)
        self.accel_gain = accel_gain
        self.brake_gain = brake_gain
        self.steer_gain = steer_gain
        self.accel=0
        self.brake=0
        self.shift_gear=2
        self.steer=0
        
    def publish_control(self,err_x, err_y):

        if err_y > 0 or err_y == 0: 
            self.accel = err_y * self.accel_gain / 240 
        else:
            self.brake = err_y * self.brake_gain/240
        self.steer = -err_x*self.steer_gain/320
        header = Header(stamp=rospy.Time.now())
        Values = Control(header, self.accel, self.brake, self.steer, self.shift_gear)
        self.pub.publish(Values)