/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Sensors.NAVX;

public class VPID extends CommandBase {
  /**
   * 
   * 
   * Creates a new VPID.
   */

  private static final double kF = 0.1;
  private static final double kP = 0.;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final int pidSlot = 0;
  private static double targetVelocityTics;
  private static double targetVelocityFT;
  private NAVX navx;


  public VPID(double ft) {
    targetVelocityFT = ft;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      navx = new NAVX(RobotMap.ahrs);
      targetVelocityTics = (DriveForward.inchesToTics(targetVelocityFT)/10);
      SmartDashboard.putNumber("targetVelocityTics", targetVelocityTics);
      RobotMap.leftMaster.config_kF(pidSlot, kF);
      RobotMap.leftMaster.config_kP(pidSlot, kP);
      RobotMap.leftMaster.config_kI(pidSlot, kI);
      RobotMap.leftMaster.config_kD(pidSlot, kD);

      RobotMap.rightMaster.config_kF(pidSlot, kF);
      RobotMap.rightMaster.config_kP(pidSlot, kP);
      RobotMap.rightMaster.config_kI(pidSlot, kI);
      RobotMap.rightMaster.config_kD(pidSlot, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotMap.rightMaster.set(ControlMode.Velocity, targetVelocityTics);
    RobotMap.leftMaster.set(ControlMode.Velocity, -targetVelocityTics);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotMap.rightMaster.set(ControlMode.Velocity, 0);
      RobotMap.leftMaster.set(ControlMode.Velocity, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(navx.currentAngle() - Robot.camera.getAngle()) <= 3) {
      return true;
    }
    
    return false;
  }
}
