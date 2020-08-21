package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.subsystems.DriveTrain;

public class RobotConfig {
    public RobotConfig() {

    }
    // adarsh has no thumbs

    public static void setStartingConfig() {

        DriveTrain.initVPID();
   
        RobotMap.leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,0,0);
        RobotMap.rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,0,0);

        RobotMap.leftMaster.setSelectedSensorPosition(0);
        RobotMap.rightMaster.setSelectedSensorPosition(0);


        //RobotMap.leftFollower1.set(ControlMode.Follower, RobotMap.leftMasterID);
        //RobotMap.rightFollower2.set(ControlMode.Follower, RobotMap.rightMasterID);
        RobotMap.leftFollower2.set(ControlMode.Follower, RobotMap.leftMasterID);
        RobotMap.rightFollower1.set(ControlMode.Follower, RobotMap.rightMasterID);
        RobotMap.leftMaster.setInverted(true);
        RobotMap.rightMaster.setInverted(false);
        //RobotMap.leftFollower1.setInverted(InvertType.FollowMaster);
        //RobotMap.rightFollower2.setInverted(InvertType.FollowMaster);
        RobotMap.leftFollower2.setInverted(InvertType.FollowMaster);
        RobotMap.rightFollower1.setInverted(InvertType.FollowMaster);

        //RobotMap.leftMaster.enableVoltageCompensation(true);

        RobotConfig.setDriveTrainVoltageCompensation();
        RobotMap.rightMaster.setSensorPhase(true);
        RobotMap.leftMaster.setSensorPhase(true);


        
    }
    public static void setDriveTrainVoltageCompensation(){
        for(TalonSRX talon:RobotMap.driveMotors){
            talon.configVoltageCompSaturation(11.0);
            talon.enableVoltageCompensation(true);
        }
    }
}