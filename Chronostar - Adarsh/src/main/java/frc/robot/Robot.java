/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotMap;
import frc.robot.Sensors.VisionCamera;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ForwardBack;
import frc.robot.commands.Turn90;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static SerialPort jevois1;
  public static VisionCamera camera;
  public static boolean hasCamera = true;
  

  //private DriveForward driveForward = new DriveForward(0.2, 2);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try {
			jevois1 = new SerialPort(115200, Port.kUSB);
      camera= new VisionCamera(Robot.jevois1);
		} catch (Exception e) {
			hasCamera = false;
		}
    
    

    
    RobotConfig.setStartingConfig();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    try {
      if(jevois1.getBytesReceived()>2){
        hasCamera = true;
      }
      else{
        hasCamera = false;
      }
      camera.updateVision();
    }
    catch(Exception e) {
      hasCamera = false;
    }
    try {
      SmartDashboard.putNumber("timesinceLast", camera.lastParseTime);
    }
    catch(Exception e) {
      
    }
    
    SmartDashboard.putNumber("LeftEncTicks", RobotMap.leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("RightEncTicks", RobotMap.rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Percent output", RobotMap.rightMaster.getMotorOutputPercent());
    double inchesMoved = (RobotMap.rightMaster.getSelectedSensorPosition(0) * 6 * Math.PI)/11264;
    SmartDashboard.putNumber("Distance moved", (inchesMoved));
    SmartDashboard.putNumber("motorVelocity", RobotMap.leftMaster.getSelectedSensorVelocity());

    //SmartDashboard.putBoolean("HasCamera", hasCamera);
    try{
      camera.updateVision();
      SmartDashboard.putNumber("cambytes", jevois1.getBytesReceived());
      SmartDashboard.putString("visionString", camera.getString());
      //SmartDashboard.putNumber("visionAngle", camera.getAngle());
    }
    catch(Exception e) {

    }
    

    

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //DriveForward driveForward = new DriveForward(120);
    //driveForward.schedule();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }


  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    //RobotMap.leftMaster.config_kF(0, (0.1 * 1023)/1030);


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    RobotMap.driveTrain.telopPeriodic();
    if(ButtonMap.xButtonPressed()) {
      //SmartDashboard.putNumber("cameraAngle", camera.getAngle());
      Turn90 turning90 = new Turn90();
      turning90.schedule();
    }
    //if(ButtonMap.aButtonPressed()) {
      //DriveTrain.setLeftSpeed(3);
      //DriveTrain.setRightSpeed(-3);
    //}
    //DriveTrain.setLeftSpeed(-3);
    //DriveTrain.setRightSpeed(3);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
