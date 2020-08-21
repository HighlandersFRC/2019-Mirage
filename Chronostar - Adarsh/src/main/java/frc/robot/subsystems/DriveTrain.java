/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ButtonMap;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {
	/**
	 * Creates a new DriveTrain.
	 */

	private double leftPower = 0;
	private double rightPower = 0;

	public DriveTrain() {

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void telopPeriodic(){
		arcadeDrive();
	}

	public void tankDrive() {
		if (Math.abs(ButtonMap.getRightSide()) > 0.1) {
			RobotMap.rightMaster.set(ControlMode.PercentOutput, ButtonMap.getRightSide());
		}

		else {
			RobotMap.rightMaster.set(ControlMode.PercentOutput, 0);
		}

		if (Math.abs(ButtonMap.getLeftSide()) > 0.1) {
			RobotMap.leftMaster.set(ControlMode.PercentOutput, ButtonMap.getLeftSide());
		}

		else {
			RobotMap.leftMaster.set(ControlMode.PercentOutput, 0);
		}
	}

	public void arcadeDrive() {
		leftPower = ButtonMap.getForwardMovement() - ButtonMap.getTurningMovement();
		rightPower = ButtonMap.getForwardMovement() + ButtonMap.getTurningMovement();
		if (Math.abs(ButtonMap.getForwardMovement()) > 0.1 || Math.abs(ButtonMap.getTurningMovement()) > 0.1) {
			if(Math.abs(leftPower) > 1) {
				leftPower = leftPower/Math.abs(leftPower);
				rightPower = rightPower/Math.abs(leftPower);
			}
			else if(Math.abs(rightPower) > 1) {
				rightPower = rightPower/Math.abs(rightPower);
				leftPower = leftPower/Math.abs(rightPower);
			}
		
		RobotMap.rightMaster.set(ControlMode.PercentOutput, rightPower);
		RobotMap.leftMaster.set(ControlMode.PercentOutput, leftPower);

			
		}

		else {
			RobotMap.leftMaster.set(ControlMode.PercentOutput, 0);
			RobotMap.rightMaster.set(ControlMode.PercentOutput, 0);
		}
 
	}

	public static void initVPID() {
		RobotMap.leftMaster.selectProfileSlot(RobotMap.pidSlot, 0);
		RobotMap.leftMaster.config_kF(RobotMap.pidSlot, RobotMap.kF);
		RobotMap.leftMaster.config_kP(RobotMap.pidSlot, RobotMap.kP);
		RobotMap.leftMaster.config_kI(RobotMap.pidSlot, RobotMap.kI);
		RobotMap.leftMaster.config_kD(RobotMap.pidSlot, RobotMap.kD);
  
		RobotMap.rightMaster.selectProfileSlot(RobotMap.pidSlot, 0);
		RobotMap.rightMaster.config_kF(RobotMap.pidSlot, RobotMap.kF);
		RobotMap.rightMaster.config_kP(RobotMap.pidSlot, RobotMap.kP);
		RobotMap.rightMaster.config_kI(RobotMap.pidSlot, RobotMap.kI);
		RobotMap.rightMaster.config_kD(RobotMap.pidSlot, RobotMap.kD);
	  }
   
	  public static void setLeftSpeed(double targetVelocityFT) {
		double targetVelocityTics = (RobotMap.ftPerSecToNIPer100ms(targetVelocityFT));
		RobotMap.leftMaster.set(ControlMode.Velocity, targetVelocityTics);
		//SmartDashboard.putNumber("tics", targetVelocityTics);
	  }
	  
	  public static void setRightSpeed(double targetVelocityFT) {
		double targetVelocityTics = (RobotMap.ftPerSecToNIPer100ms(targetVelocityFT));
		RobotMap.rightMaster.set(ControlMode.Velocity, targetVelocityTics);
	  }

}
