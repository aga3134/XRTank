#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from xr_tank.msg import ArmJoyCmd
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs
import actionlib
from copy import copy
from std_msgs.msg import Header
import math

class ArmControl():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.armGroup = moveit_commander.MoveGroupCommander("arm")
        self.planningFrame = self.armGroup.get_planning_frame()
        self.endEffector = self.armGroup.get_end_effector_link()

        self.armJoyCmdSub = rospy.Subscriber("/arm_joy_cmd", ArmJoyCmd, self.ArmJoyCmdCallback)
        self.armAC = actionlib.SimpleActionClient("/xr_tank/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.armAC.wait_for_server(timeout=rospy.Duration(10.0))
        self.jointState = self.armGroup.get_current_joint_values()

        #add ground plane
        ground_pose = geometry_msgs.msg.PoseStamped()
        ground_pose.header.frame_id = "vehicle_base"
        ground_pose.pose.orientation.w = 1.0
        ground_pose.pose.position.z = -0.05
        ground_name = "ground"
        self.scene.add_box(ground_name, ground_pose, size=(1, 1, 0.1))

        #self.trajectoryPublisher = rospy.Publisher('/arm/planned_path',DisplayTrajectory, queue_size=20)
        #rospy.loginfo(self.planningFrame)
        #rospy.loginfo(self.endEffector)
        #rospy.loginfo(self.robot.get_group_names())
        #rospy.loginfo(self.armGroup.get_current_joint_values())

    def ArmJoyCmdCallback(self,msg):
        #rospy.loginfo(msg)
        if msg.resetPose:
            self.GoToStandbyPose()
        elif msg.randomPose:
            self.GoToRandomPose()
        elif msg.armAOffset != 0 or msg.armBOffset != 0 or msg.gripperBaseOffset != 0:
            #target = self.armGroup.get_current_joint_values()
            #rospy.loginfo("%f %f %f %f" % (target[0],target[1],target[2],target[3]))
            

            #send action goal directly from action client to skip motion planning
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["joint_arm_a","joint_arm_b","joint_gripper_base"]
            #first point (original pose)
            point = JointTrajectoryPoint()
            point.positions = copy(self.jointState)
            point.time_from_start = rospy.Duration(0)
            goal.trajectory.points.append(point)
            
            #second point(pose after moving by joystick)
            scale = 0.05
            self.jointState[0] += msg.armAOffset*scale
            self.jointState[1] += msg.armBOffset*scale
            self.jointState[2] += msg.gripperBaseOffset*scale
            point = JointTrajectoryPoint()
            point.positions = copy(self.jointState)
            point.time_from_start = rospy.Duration(0.03)
            goal.trajectory.points.append(point)
            
            #rospy.loginfo(goal)
            self.armAC.send_goal(goal)
            #self.armAC.wait_for_result(timeout=rospy.Duration(1))
        

    def GoToRandomPose(self):
        rospy.loginfo("======go to random pose")
        #pose_goal = self.armGroup.get_random_pose()
        #self.armGroup.set_pose_target(pose_goal)
        #self.armGroup.go(wait=True)

        joint_goal = self.armGroup.get_random_joint_values()
        self.jointState = copy(joint_goal)
        self.armGroup.go(joint_goal, wait=True)
        self.armGroup.stop()

    def GoToStandbyPose(self):
        rospy.loginfo("======go to standby pose")
        joint_goal = self.armGroup.get_named_target_values("standby")
        #rospy.loginfo(joint_goal)
        self.jointState = [joint_goal["joint_arm_a"],joint_goal["joint_arm_b"],joint_goal["joint_gripper_base"]]
        self.armGroup.go(joint_goal, wait=True)
        self.armGroup.stop()

    def Run(self):
        rospy.spin()
        """rate = rospy.Rate(0.2)
        mode = 0
        while not rospy.is_shutdown():
            rate.sleep()
            if mode % 2 == 0:
                self.GoToRandomPose()
            if mode % 2 == 1:
                self.GoToStandbyPose()
        """

        

if __name__ == '__main__':
    rospy.init_node('arm_control_node')
    rospy.loginfo("arm_control_node started")
    armControl = ArmControl()
    armControl.Run()
