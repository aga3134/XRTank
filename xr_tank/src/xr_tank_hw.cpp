#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include "xr_tank/ArmPose.h"
#include "xr_tank/CamPanTilt.h"
#include "xr_tank/WheelDrive.h"

#define PI 3.1415926
#define DEG2RAD PI/180
#define ARM_A 0
#define ARM_B 1
#define GRIPPER_BASE 2
#define GRIPPER 3
#define CAM_PAN 4
#define CAM_TILT 5
#define JOINT_NUM 6

#define WHEEL_L 0
#define WHEEL_R 1
#define WHEEL_NUM 2

class XRTankHW : public hardware_interface::RobotHW{
    public:
        XRTankHW(ros::NodeHandle nh): m_NH(nh){
            for(int i=0;i<JOINT_NUM;i++){
                m_JointPose[i] = 0;
                m_JointVel[i] = 0;
                m_JointEff[i] = 0;
            }
            for(int i=0;i<WHEEL_NUM;i++){
                m_WheelPose[i] = 0;
                m_WheelVel[i] = 0;
                m_WheelEff[i] = 0;
            }
            //position controller: arm, cam pan tilt
            //joint state
            const char* joint[] = {"joint_arm_a","joint_arm_b","joint_gripper_base","joint_gripper_l","joint_cam_pan","joint_cam_tilt"};
            for(int i=0;i<JOINT_NUM;i++){
                hardware_interface::JointStateHandle stateHandle(joint[i], &m_JointPose[i], &m_JointVel[i], &m_JointEff[i]);
                m_JointStateInterface.registerHandle(stateHandle);
            }
            //position
            ros::NodeHandle limitNH("/robot_description_planning");
            for(int i=0;i<JOINT_NUM;i++){
                hardware_interface::JointHandle posHandle(m_JointStateInterface.getHandle(joint[i]), &m_JointCmd[i]);
                m_PositionJointInterface.registerHandle(posHandle);

                joint_limits_interface::JointLimits limits;
                joint_limits_interface::SoftJointLimits softLimits;
                if (getJointLimits(joint[i], limitNH, limits) == false) {
                    ROS_ERROR_STREAM("Cannot set joint limits for " << joint[i]);
                } else {
                    ROS_INFO("joint limit %s: %f %f", joint[i], limits.min_position, limits.max_position);
                    softLimits.k_position = 1;
                    softLimits.min_position = limits.min_position;
                    softLimits.max_position = limits.max_position;
                    joint_limits_interface::PositionJointSoftLimitsHandle jointLimitsHandle(posHandle, limits, softLimits);
                    m_PositionJointLimitInterface.registerHandle(jointLimitsHandle);
                }
            }

            //effort controller: wheel
            //joint state
            const char* wheel[] = {"joint_wheel_l","joint_wheel_r"};
            for(int i=0;i<WHEEL_NUM;i++){
                hardware_interface::JointStateHandle stateHandle(wheel[i], &m_WheelPose[i], &m_WheelVel[i], &m_WheelEff[i]);
                m_JointStateInterface.registerHandle(stateHandle);
            }
            //effort
            for(int i=0;i<WHEEL_NUM;i++){
                hardware_interface::JointHandle effortHandle(m_JointStateInterface.getHandle(wheel[i]), &m_WheelCmd[i]);
                m_EffortJointInterface.registerHandle(effortHandle);
            }

            registerInterface(&m_JointStateInterface);
            registerInterface(&m_PositionJointInterface);
            registerInterface(&m_EffortJointInterface);
            registerInterface(&m_PositionJointLimitInterface);


            m_ArmCmdPub = m_NH.advertise<xr_tank::ArmPose>("/arm_pose_cmd", 1);
            m_CamPanTiltPub = m_NH.advertise<xr_tank::CamPanTilt>("/cam_pan_tilt", 1);
            m_WheelDrivePub = m_NH.advertise<xr_tank::WheelDrive>("/wheel_drive", 1);
        }

        void read(const ros::Time& time, const ros::Duration& period){
            
        }

        void write(const ros::Time& time, const ros::Duration& period){
            //ROS_INFO("arm cmd: %lf %lf %lf %lf %lf %lf", m_JointCmd[0],m_JointCmd[1],m_JointCmd[2],m_JointCmd[3],m_JointCmd[4],m_JointCmd[5]);
            m_PositionJointLimitInterface.enforceLimits(period);

            //ROS_INFO("arm cmd after limit: %lf %lf %lf %lf %lf %lf", m_JointCmd[0],m_JointCmd[1],m_JointCmd[2],m_JointCmd[3],m_JointCmd[4],m_JointCmd[5]);
            xr_tank::ArmPose armPose;
            armPose.armAPos = m_JointCmd[ARM_A];
            armPose.armBPos = m_JointCmd[ARM_B];
            armPose.gripperBasePos = m_JointCmd[GRIPPER_BASE];
            armPose.gripperPos = m_JointCmd[GRIPPER];
            m_ArmCmdPub.publish(armPose);

            xr_tank::CamPanTilt panTilt;
            panTilt.panPos = m_JointCmd[CAM_PAN];
            panTilt.tiltPos = m_JointCmd[CAM_TILT];
            //ROS_INFO("cmd %d %d", panTilt.panPos, panTilt.tiltPos);
            m_CamPanTiltPub.publish(panTilt);
            //pass cmd to state since we don't have pos feedback
	    m_JointPose[ARM_A] = m_JointCmd[ARM_A];
	    m_JointPose[ARM_B] = m_JointCmd[ARM_B];
	    m_JointPose[GRIPPER_BASE] = m_JointCmd[GRIPPER_BASE];
	    m_JointPose[GRIPPER] = m_JointCmd[GRIPPER];
            m_JointPose[CAM_PAN] = m_JointCmd[CAM_PAN];
            m_JointPose[CAM_TILT] = m_JointCmd[CAM_TILT];

            xr_tank::WheelDrive wheelDrive;
            wheelDrive.leftSpeed = m_WheelCmd[WHEEL_L];
            wheelDrive.rightSpeed = m_WheelCmd[WHEEL_R];
            m_WheelDrivePub.publish(wheelDrive);
            //pass cmd to state since we don't have wheel drive feedback
            m_WheelPose[WHEEL_L] = m_WheelCmd[WHEEL_L];
            m_WheelPose[WHEEL_R] = m_WheelCmd[WHEEL_R];
        }

    private:
        ros::NodeHandle m_NH;
        ros::Publisher m_ArmCmdPub,m_CamPanTiltPub,m_WheelDrivePub;

        hardware_interface::JointStateInterface m_JointStateInterface;
        hardware_interface::PositionJointInterface m_PositionJointInterface;
        hardware_interface::EffortJointInterface m_EffortJointInterface;
        joint_limits_interface::PositionJointSoftLimitsInterface m_PositionJointLimitInterface;
        
        double m_JointCmd[JOINT_NUM];
        double m_JointPose[JOINT_NUM];
        double m_JointVel[JOINT_NUM];
        double m_JointEff[JOINT_NUM];

        double m_WheelCmd[WHEEL_NUM];
        double m_WheelPose[WHEEL_NUM];
        double m_WheelVel[WHEEL_NUM];
        double m_WheelEff[WHEEL_NUM];
};

int main(int argc, char** argv){
    ros::init(argc, argv, "xr_tank_hw");
    ros::NodeHandle nh;

    XRTankHW robot(nh);
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate r(30);
    ros::Time lastTime = ros::Time::now();
    while (ros::ok()){
        ros::Time curTime = ros::Time::now();
        ros::Duration duration = curTime - lastTime;
        //robot.read(curTime,duration);
        cm.update(curTime, duration);
        robot.write(curTime, duration);
        r.sleep();
    }

    return 0;
}
