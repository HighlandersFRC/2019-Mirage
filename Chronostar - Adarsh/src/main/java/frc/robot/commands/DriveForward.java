/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Tools.controlLoops.PID;

public class DriveForward extends CommandBase {
  /**
   * Creates a new DriveForward.
   */

  private PID pid;


  private final double kF = 0.163;
  private final double kP = 0.0002;
  private final double kI = 0.0000025;
  private final double kD = 0.00004;
  private final int cruiseVelocity = 3000;
  private final int acceleration = 6000;
  private final int pidSlot = 0;
  private double targetPos;
  private int targetTics;
  //private double targetInches;
  public DriveForward(double targetPosition) {
    //desiredSpeed = speed;
    //desiredTime = time;
    targetPos = targetPosition;
    targetTics = inchesToTics(targetPos);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static double ticsToInches(double encoderTics) {
    double inches = (encoderTics * 6 * Math.PI)/11264;
    return inches;
  }

  public static int inchesToTics(double inches) {
    int encoderTics = (int) Math.round((inches * 11264) / 6);
    encoderTics = (int) Math.round(encoderTics/Math.PI);
    return encoderTics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //initTime = Timer.getFPGATimestamp();
    //targetPos = (3 * 2048 * 6 * Math.PI)/12;
    RobotMap.leftMaster.setSelectedSensorPosition(0);
    RobotMap.rightMaster.setSelectedSensorPosition(0); 
    System.out.println("Inside initialize");
    
    RobotMap.leftMaster.config_kF(pidSlot, kF);
    RobotMap.leftMaster.config_kP(pidSlot, kP);
    RobotMap.leftMaster.config_kI(pidSlot, kI);
    RobotMap.leftMaster.config_kD(pidSlot, kD);

    RobotMap.rightMaster.config_kF(pidSlot, kF);
    RobotMap.rightMaster.config_kP(pidSlot, kP);
    RobotMap.rightMaster.config_kI(pidSlot, kI);
    RobotMap.rightMaster.config_kD(pidSlot, kD);

    RobotMap.rightMaster.configMotionCruiseVelocity(cruiseVelocity);
    RobotMap.leftMaster.configMotionCruiseVelocity(cruiseVelocity);

    RobotMap.rightMaster.configMotionAcceleration(acceleration);
    RobotMap.leftMaster.configMotionAcceleration(acceleration);

    RobotMap.rightMaster.set(ControlMode.MotionMagic, targetTics);
    RobotMap.leftMaster.set(ControlMode.MotionMagic, targetTics);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("leftPercent"+ RobotMap.leftMaster.getControlMode());
    SmartDashboard.putNumber("target",  targetTics);
    SmartDashboard.putNumber("rightDist", RobotMap.rightMaster.getSelectedSensorPosition(0));


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotMap.rightMaster.set(ControlMode.PercentOutput, 0);
    RobotMap.leftMaster.set(ControlMode.PercentOutput, 0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(ticsToInches(RobotMap.rightMaster.getSelectedSensorPosition(0))- targetTics) < 1000){
      return true;
    }
    return false;
  }
}
