/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*---------------------------------------------------------------------------*/

package frc.robot.commands.humanInterface;


import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.commands.autos.DriveForward;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ButtonMap;

public class DriveInterface extends Command {
	private boolean shouldFinish;
	private DriveForward driveForward;
	public DriveInterface() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(RobotMap.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		RobotMap.drive.initVelocityPIDs();
		driveForward = new DriveForward();
	}


	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if(ButtonMap.runAuto()&&!driveForward.isRunning()){
			driveForward.start();
		}
		else{
			RobotMap.drive.arcadeDrive();
		}
	}
	public void forceEnd(){
		shouldFinish = true;
	}
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return RobotState.isDisabled()||shouldFinish;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		RobotMap.drive.stopDriveTrainMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		this.end();
	}
}
