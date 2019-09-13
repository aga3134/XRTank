#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from xr_tank.msg import ArmPose,CamPanTilt,WheelDrive

import RPi.GPIO as GPIO
import time
import math

JOINT_CAM_PAN=31
JOINT_CAM_TILT=32

JOINT_ARM_A=23
JOINT_ARM_B=24
JOINT_GRIPPER_BASE=26
JOINT_GRIPPER=29

RIGHT_FORWARD=35
RIGHT_BACKWARD=36
LEFT_FORWARD=40
LEFT_BACKWARD=37
ENABLE_A=33
ENABLE_B=38

class XRDriver():
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(JOINT_CAM_PAN,GPIO.OUT)
        GPIO.setup(JOINT_CAM_TILT,GPIO.OUT)
        GPIO.setup(JOINT_ARM_A,GPIO.OUT)
        GPIO.setup(JOINT_ARM_B,GPIO.OUT)
        GPIO.setup(JOINT_GRIPPER_BASE,GPIO.OUT)
        GPIO.setup(JOINT_GRIPPER,GPIO.OUT)
        GPIO.setup(RIGHT_FORWARD,GPIO.OUT)
        GPIO.setup(RIGHT_BACKWARD,GPIO.OUT)
        GPIO.setup(LEFT_FORWARD,GPIO.OUT)
        GPIO.setup(LEFT_BACKWARD,GPIO.OUT)
        GPIO.setup(ENABLE_A,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(ENABLE_B,GPIO.OUT,initial=GPIO.HIGH)

        freq = 500
        self.camPan = GPIO.PWM(JOINT_CAM_PAN,freq)
        self.camTilt = GPIO.PWM(JOINT_CAM_TILT,freq)
        self.armA = GPIO.PWM(JOINT_ARM_A,freq)
        self.armB = GPIO.PWM(JOINT_ARM_B,freq)
        self.gripperBase = GPIO.PWM(JOINT_GRIPPER_BASE,freq)
        self.gripper = GPIO.PWM(JOINT_GRIPPER,freq)
        self.rightForward = GPIO.PWM(RIGHT_FORWARD,freq)
        self.rightBackward = GPIO.PWM(RIGHT_BACKWARD,freq)
        self.leftForward = GPIO.PWM(LEFT_FORWARD,freq)
        self.leftBackward = GPIO.PWM(LEFT_BACKWARD,freq)

        self.camPanMin = rospy.get_param("~camPanMin",-0.5*math.pi)
        self.camPanMax = rospy.get_param("~camPanMax",math.pi/6)
        #self.camPanPWMInit = rospy.get_param("~camPanPWMInit",85)
        self.camPanPWMMin = rospy.get_param("~camPanPWMMin",20)
        self.camPanPWMMax = rospy.get_param("~camPanPWMMax",99)
        
        self.camTiltMin = rospy.get_param("~camTiltMin",-0.5*math.pi)
        self.camTiltMax = rospy.get_param("~camTiltMax",0)
        #self.camTiltPWMInit = rospy.get_param("~camTiltPWMInit",20)
        self.camTiltPWMMin = rospy.get_param("~camTiltPWMMin",99)
        self.camTiltPWMMax = rospy.get_param("~camTiltPWMMax",20)
        
        self.armAMin = rospy.get_param("~armAMin",-math.pi/6)
        self.armAMax = rospy.get_param("~armAMax",0.5*math.pi)
        #self.armAPWMInit = rospy.get_param("~armAPWMInit",70)
        self.armAPWMMin = rospy.get_param("~armAPWMMin",99)
        self.armAPWMMax = rospy.get_param("~armAPWMMax",20)
        
        self.armBMin = rospy.get_param("~armBMin",0)
        self.armBMax = rospy.get_param("~armBMax",math.pi*2/3)
        #self.armBPWMInit = rospy.get_param("~armBPWMInit",99)
        self.armBPWMMin = rospy.get_param("~armBPWMMin",99)
        self.armBPWMMax = rospy.get_param("~armBPWMMax",20)
        
        self.gripperBaseMin = rospy.get_param("~gripperBaseMin",-0.5*math.pi)
        self.gripperBaseMax = rospy.get_param("~gripperBaseMax",0.5*math.pi)
        #self.gripperBasePWMInit = rospy.get_param("~gripperBasePWMInit",70)
        self.gripperBasePWMMin = rospy.get_param("~gripperBasePWMMin",99)
        self.gripperBasePWMMax = rospy.get_param("~gripperBasePWMMax",20)
        
        self.gripperMin = rospy.get_param("~gripperMin",0)
        self.gripperMax = rospy.get_param("~gripperMax",0.03)
        #self.gripperPWMInit = rospy.get_param("~gripperPWMInit",99)
        self.gripperPWMMin = rospy.get_param("~gripperPWMMin",99)
        self.gripperPWMMax = rospy.get_param("~gripperPWMMax",70)
        
        
        self.camPan.start(0)
        self.camTilt.start(0)
        self.armA.start(0)
        self.armB.start(0)
        self.gripperBase.start(0)
        self.gripper.start(0)
        

        self.echoPub = rospy.Publisher("/driver_echo",String,queue_size=1)
        self.wheelDriveSub = rospy.Subscriber("/wheel_drive",WheelDrive,self.WheelDriveCB)
        self.camPanTiltSub = rospy.Subscriber("/cam_pan_tilt",CamPanTilt,self.CamPanTiltCB)
        self.armPoseCmdSub = rospy.Subscriber("/arm_pose_cmd",ArmPose,self.ArmPoseCmdCB)

    def __del__(self):
        self.camPan.stop()
        self.camTilt.stop()
        self.armA.stop()
        self.armB.stop()
        self.gripperBase.stop()
        self.gripper.stop()
        self.rightForward.stop()
        self.rightBackward.stop()
        self.leftForward.stop()
        self.leftBackward.stop()
        GPIO.cleanup()

    def PosToPWM(self,minPos,maxPos,minPWM,maxPWM,pos):
        alpha = (pos-minPos)/(maxPos-minPos)
        pwm = alpha*(maxPWM-minPWM)+minPWM
        if(pwm > 100):
            pwm = 100
        if(pwm < 0):
            pwm = 0
        return pwm

    def WheelDriveCB(self,msg):
        #self.echoPub.publish("recieve wheel drive message")
        if(msg.leftSpeed>0):
            self.leftForward.ChangeDutyCycle(msg.leftSpeed)
            self.leftBackward.ChangeDutyCycle(0)
        else:
            self.leftForward.ChangeDutyCycle(0)
            self.leftBackward.ChangeDutyCycle(-msg.leftSpeed)
        
        if(msg.rightSpeed>0):
            self.rightForward.ChangeDutyCycle(msg.rightSpeed)
            self.rightBackward.ChangeDutyCycle(0)
        else:
            self.rightForward.ChangeDutyCycle(0)
            self.rightBackward.ChangeDutyCycle(-msg.rightSpeed)
        
    def CamPanTiltCB(self,msg):
        #self.echoPub.publish("recieve cam pan tilt message")
        panPWM = self.PosToPWM(self.camPanMin,self.camPanMax,self.camPanPWMMin, self.camPanPWMMax, msg.panPos)
        self.camPan.ChangeDutyCycle(panPWM)
        tiltPWM = self.PosToPWM(self.camTiltMin,self.camTiltMax,self.camTiltPWMMin, self.camTiltPWMMax, msg.tiltPos)
        self.camTilt.ChangeDutyCycle(tiltPWM)

    def ArmPoseCmdCB(self,msg):
        #self.echoPub.publish("recieve arm pose cmd message")
        armAPWM = self.PosToPWM(self.armAMin,self.armAMax,self.armAPWMMin, self.armAPWMMax, msg.armAPos)
        #self.armA.ChangeDutyCycle(armAPWM)
        armBPWM = self.PosToPWM(self.armBMin,self.armBMax,self.armBPWMMin, self.armBPWMMax, msg.armBPos)
        #self.armB.ChangeDutyCycle(armBPWM)
        gripperBasePWM = self.PosToPWM(self.gripperBaseMin,self.gripperBaseMax,self.gripperBasePWMMin, self.gripperBasePWMMax, msg.gripperBasePos)
        #self.gripperBase.ChangeDutyCycle(gripperBasePWM)
        gripperPWM = self.PosToPWM(self.gripperMin,self.gripperMax,self.gripperPWMMin, self.gripperPWMMax, msg.gripperPos)
        #self.gripper.ChangeDutyCycle(gripperPWM)


if __name__ == '__main__':
    rospy.init_node('xr_driver_node')
    rospy.loginfo("xr_driver_node started")
    driver = XRDriver()
    rospy.spin()
