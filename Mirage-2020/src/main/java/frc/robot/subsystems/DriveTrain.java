package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.RobotStats;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

   public static OI oi = new OI();

  public DriveTrain() {
      


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void teleopPeriodic(){

      tankDrive();
  }
  public static void initVPID(){
    RobotMap.leftMaster.selectProfileSlot(0, 0);
    RobotMap.leftMaster.config_kF(0, 0.32);
    RobotMap.leftMaster.config_kP(0,0);
    RobotMap.leftMaster.config_kI(0,0); 
    RobotMap.leftMaster.config_kD(0,0);
    RobotMap.rightMaster.selectProfileSlot(0,0); 
    RobotMap.rightMaster.config_kF(0, 0.32);
    RobotMap.rightMaster.config_kP(0,0);
    RobotMap.rightMaster.config_kI(0,0);
    RobotMap.rightMaster.config_kD(0,0);
    RobotMap.rightMaster.configMotionCruiseVelocity(900);
    RobotMap.leftMaster.configMotionCruiseVelocity(900);
    RobotMap.rightMaster.configMotionAcceleration(1800);                        
    RobotMap.leftMaster.configMotionAcceleration(1800);

  }
  public static void setSpeed(double targetVelocityFeet){
    double targetVelocityTicks = RobotStats.inchesToTicks(targetVelocityFeet) / 12;
    RobotMap.leftMaster.set(ControlMode.Velocity, targetVelocityTicks);
    RobotMap.rightMaster.set(ControlMode.Velocity, targetVelocityTicks);
  }
  public void tankDrive(){
    double turn;

    if(Math.abs(oi.driverController.getRawAxis(4)) > 0.1) {
        turn = (oi.driverController.getRawAxis(4));
    }
    else {
        turn = 0; 
    }


      if(Math.abs(oi.driverController.getRawAxis(1)) > 0.1) {
          RobotMap.rightMaster.set(ControlMode.PercentOutput,oi.driverController.getRawAxis(1)+turn);
          RobotMap.leftMaster.set(ControlMode.PercentOutput,oi.driverController.getRawAxis(1)-turn);
      }
      else {
          RobotMap.rightMaster.set(ControlMode.PercentOutput,turn);
          RobotMap.leftMaster.set(ControlMode.PercentOutput,-turn);
        
      }

     
      }

      


      
      }
      

      
      
  

