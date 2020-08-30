/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ButtonMap;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.RobotStats;

public class DriveForwards extends Command {
  public DriveForwards() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.leftMaster.selectProfileSlot(0,0);
    RobotMap.rightMaster.selectProfileSlot(0,0);
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    RobotMap.leftMaster.set(ControlMode.MotionMagic, RobotStats.inchesToTicks(24));
    RobotMap.rightMaster.set(ControlMode.PercentOutput, RobotStats.inchesToTicks(24));

  }

  // Make this return true when this Command no longer needs to run execute()
 

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
  }

  @Override
  public boolean isFinished() {
    if(OI.driverController.getXButton()){
      return true;
    }
    return false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

