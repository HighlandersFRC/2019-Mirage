/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ButtonMap;
import frc.robot.RobotMap;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
 

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private double shooterPower;
  private double kf = 0.00017;
  private double kp = 0;
  private double ki = 0;
  private double kd = 0;
  private CANPIDController vpidController = new CANPIDController(RobotMap.shooterMotorOne);

  public Shooter() {


  }
  public void initShooterPID(){
  shooterPower = 0;
  vpidController.setFF(kf);
  vpidController.setP(kp);
  vpidController.setI(ki);
  vpidController.setD(kd);
  vpidController.setOutputRange(0, 1);  
  RobotMap.shooterMotorTwo.follow(RobotMap.shooterMotorOne, true);
  }

  @Override
  public void periodic() {
    if(ButtonMap.shootyUp()){
      shooterPower = shooterPower +100;
    }
    else if(ButtonMap.shootyDown()){
      shooterPower = shooterPower-100;
    }
    else if(ButtonMap.shootyStop()){
      shooterPower = 0;
    }
    if(shooterPower >4500){
      shooterPower = 4500;
    }
    if(shooterPower <0){
      shooterPower = 0;
    }
    
    SmartDashboard.putNumber("velocity",RobotMap.shooterMotorOne.getEncoder().getVelocity());
    SmartDashboard.putNumber("desiredVelocity",shooterPower);

    vpidController.setReference(shooterPower, ControlType.kVelocity);
    // This method will be called once per scheduler run
  }
}
