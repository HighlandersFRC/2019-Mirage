/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.RobotStats;

public class MotionMagicForward extends CommandBase {
  /**
   * Creates a new MotionMagicForward.
   */
  public MotionMagicForward() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotMap.leftMaster.setSelectedSensorPosition(0); 
    RobotMap.rightMaster.setSelectedSensorPosition(0);
    RobotMap.leftMaster.selectProfileSlot(0,0);
    RobotMap.rightMaster.selectProfileSlot(0,0);
    RobotMap.leftMaster.set(ControlMode.MotionMagic, RobotStats.inchesToTicks(12));
    RobotMap.rightMaster.set(ControlMode.MotionMagic, RobotStats.inchesToTicks(12));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(OI.driverController.getXButton()){
      return true;
    }
    return false;
  }
}
