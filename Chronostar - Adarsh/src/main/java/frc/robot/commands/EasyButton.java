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
import frc.robot.RobotMap;

public class EasyButton extends CommandBase {
  /**
   * Creates a new EasyButton.
   */
  private final double kF = 0.163;
  private final double kP = 0.0002;
  private final double kI = 0.0000025;
  private final double kD = 0.00004;
  private final int pidSlot = 0;
  private double targetVelocity;

  public EasyButton(double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetVelocity = velocity;
  }


  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotMap.leftMaster.config_kF(pidSlot, kF);
    RobotMap.leftMaster.config_kP(pidSlot, kP);
    RobotMap.leftMaster.config_kI(pidSlot, kI);
    RobotMap.leftMaster.config_kD(pidSlot, kD);

    RobotMap.rightMaster.config_kF(pidSlot, kF);
    RobotMap.rightMaster.config_kP(pidSlot, kP);
    RobotMap.rightMaster.config_kI(pidSlot, kI);
    RobotMap.rightMaster.config_kD(pidSlot, kD);

    RobotMap.rightMaster.set(ControlMode.Velocity, targetVelocity);
    RobotMap.leftMaster.set(ControlMode.Velocity, targetVelocity);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("leftPercent"+ RobotMap.leftMaster.getControlMode());
    SmartDashboard.putNumber("target",  targetVelocity);
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
    return false;
  }
}
