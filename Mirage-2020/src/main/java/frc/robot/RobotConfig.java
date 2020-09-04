package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

public class RobotConfig {
    public RobotConfig(){

    }
    public static void setStartingConfig(){
        RobotMap.leftMaster.setInverted(true);
        RobotMap.rightMaster.setInverted(false);
        RobotMap.leftFollower.set(ControlMode.Follower,RobotMap.leftMasterid);
        RobotMap.leftFollower.setInverted(InvertType.FollowMaster);
        RobotMap.rightFollower.set(ControlMode.Follower,RobotMap.rightMasterid);
        RobotMap.rightFollower.setInverted(InvertType.FollowMaster);
        RobotMap.leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RobotMap.rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        RobotMap.leftMaster.setSelectedSensorPosition(0);
        RobotMap.rightMaster.setSelectedSensorPosition(0);
        RobotMap.rightMaster.setSensorPhase(true);
        RobotMap.leftMaster.setSensorPhase(true);
        
    }
}