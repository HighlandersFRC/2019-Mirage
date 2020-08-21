/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ButtonMap;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Sensors.NAVX;
import frc.robot.Tools.controlLoops.PID;
import frc.robot.subsystems.DriveTrain;

public class Turn90 extends CommandBase {
  /**
   * Creates a new Turn90.
   */

  private PID negPID;
  private PID posPID;
  //0.52
  private final double kP = 0.1;
  private final double kI = 0.0004;
  private final double kD = 0.00005;

  private final double poskP = 0.14;
  private final double poskI = 0.038;
  private final double poskD = 0.0001;

  private final double navxOffset = -4.5;

  private NAVX navx;

  private double originalAngle;

  public double jevoisAngle;

  private double startingTime;

  public Turn90() {
    // Use addRequirements() here to declare subsystem dependencies.
    //originalAngle = firstAngle;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navx = new NAVX(RobotMap.ahrs);
    //navx.softResetAngle();
    negPID = new PID(kP, kI, kD);
    negPID.setSetPoint(0);
    negPID.setMinOutput(-3);
    negPID.setMaxOutput(3);
    posPID = new PID(poskP, poskI, poskD);
    posPID.setSetPoint(0);
    posPID.setMinOutput(-3);
    posPID.setMaxOutput(3);
    RobotMap.visionRelay1.set(Value.kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try{
      startingTime = Timer.getFPGATimestamp();
      Robot.camera.updateVision();
      jevoisAngle = Robot.camera.getAngle();
      SmartDashboard.putNumber("Jevois Angle", jevoisAngle);
      SmartDashboard.putNumber("Get result", negPID.getResult());
      if(Timer.getFPGATimestamp()-Robot.camera.lastParseTime<0.25){
        negPID.updatePID(jevoisAngle + 9.5);
        //System.out.println("hi");
        DriveTrain.setLeftSpeed(-negPID.getResult());
        DriveTrain.setRightSpeed(negPID.getResult());
      }
      else{
        //System.out.println("hewwo");
        negPID.updatePID(0);
        DriveTrain.setLeftSpeed(0);
        DriveTrain.setRightSpeed(0);
      }     
    
  }
  catch(Exception e) {

  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putBoolean("Finished", false);
    RobotMap.rightMaster.set(ControlMode.PercentOutput, 0);
    RobotMap.leftMaster.set(ControlMode.PercentOutput, 0);
    RobotMap.visionRelay1.set(Value.kReverse); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!OI.driverController.getXButton()){
      return true;
    }
    //System.out.println("Still running command");
    return false;
    

  }
}
