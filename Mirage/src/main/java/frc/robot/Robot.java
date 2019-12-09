package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.tools.pathTools.Odometry;
import frc.robot.tools.pathTools.PathList;
import frc.robot.tools.utility.ControlLooper;

public class Robot extends TimedRobot {

  public static final Elevator elevator = new Elevator();

  public static OI m_oi;
  Command m_autonomousCommand;
  public static PathList pathlist = new PathList();
  private CommandSuites commandSuites;
  private RobotConfig robotConfig;

  public static final long periodMS = 20;
	public static final ControlLooper controlLoop = new ControlLooper("Main control loop", periodMS);

  @Override
  public void robotInit() {
    commandSuites = new CommandSuites();
    robotConfig = new RobotConfig();
    robotConfig.setStartingConfig();
    RobotMap.drive.initVelocityPIDs();
    m_oi = new OI();

    updateStatus();
    
    try {   
      controlLoop.addLoopable(elevator);

        } catch (Exception e) {
          System.err.println("An error occurred in robotInit()");
        }
  }

  @Override
  public void robotPeriodic() {
    updateStatus();
  }

  @Override
  public void disabledInit() {
    updateStatus();
  }
  @Override
  public void disabledPeriodic() {
    updateStatus();
    Scheduler.getInstance().run();
  }
  
  @Override
  public void autonomousInit() {
    controlLoop.start();
    elevator.targetPositionInchesMM = elevator.motor1.getPositionWorld();

		robotConfig.setAutoConfig();
    commandSuites.startAutoCommands();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
      updateStatus();
    }
  }
  
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    updateStatus();
  }
  @Override
  public void teleopInit() {
    controlLoop.start();
    elevator.targetPositionInchesMM = elevator.motor1.getPositionWorld();
    commandSuites.startTeleopCommands();
    robotConfig.setTeleopConfig();
    updateStatus();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    updateStatus();
  }

  @Override
  public void testPeriodic() {
    updateStatus();
  }


  public void updateStatus() {
		elevator.updateStatus();

   }
}
