#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from xr_tank.msg import ArmPose,CamPanTilt,WheelDrive

class XRDriver():
    def __init__(self):
        self.echoPub = rospy.Publisher("/driver_echo",String,queue_size=1)
        self.wheelDriveSub = rospy.Subscriber("/wheel_drive",WheelDrive,self.WheelDriveCB)
        self.camPanTiltSub = rospy.Subscriber("/cam_pan_tilt",CamPanTilt,self.CamPanTiltCB)
        self.armPoseCmdSub = rospy.Subscriber("/arm_pose_cmd",ArmPose,self.ArmPoseCmdCB)

    def WheelDriveCB(self,msg):
        self.echoPub.publish("recieve wheel drive message")

    def CamPanTiltCB(self,msg):
        self.echoPub.publish("recieve cam pan tilt message")

    def ArmPoseCmdCB(self,msg):
        self.echoPub.publish("recieve arm pose cmd message")


if __name__ == '__main__':
    rospy.init_node('xr_driver_node')
    rospy.loginfo("xr_driver_node started")
    driver = XRDriver()
    rospy.spin()
