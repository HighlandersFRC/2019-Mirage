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
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.tools.controlLoops.PID;

public class Turn90 extends CommandBase {
  /**
   * Creates a new Turn90.
   */
    private PID pid;
    private double kP = 0.013;
    private double kI = 0.00035;
    private double kD = 0.0013;
    


  public Turn90() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotMap.navx.softResetAngle();
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(90);
    pid.setMinOutput(-0.5);
    pid.setMaxOutput(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.updatePID(RobotMap.navx.currentAngle());
    SmartDashboard.putNumber("Current Angle", RobotMap.navx.currentAngle());
    SmartDashboard.putNumber("Result", pid.getResult());
    SmartDashboard.putBoolean("Finished", false);
    RobotMap.rightMaster.set(ControlMode.PercentOutput, pid.getResult());
    RobotMap.leftMaster.set(ControlMode.PercentOutput, -pid.getResult());
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
   if(Math.abs(RobotMap.navx.currentAngle() - 90) <= 0.5 && pid.getResult() <= 0.2){
      SmartDashboard.putBoolean("Finished", true);

      return true;
    }
     else if(OI.driverController.getYButton()){ 

      return true;
    }
    
    return false;
  }
}
