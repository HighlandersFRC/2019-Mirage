/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ButtonMap;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private boolean enableClimber = false;
  /**
   * Creates a new ClimberMechanism.
   */
  public Climber() {

  }
  public void moveClimber(){
    RobotMap.climberMaster.set(ControlMode.PercentOutput, ButtonMap.moveClimberArmUp());
  }

  @Override
  public void periodic() {
    if(RobotState.isOperatorControl()){
      if(ButtonMap.enableClimberFunctions()){
        enableClimber = true; 
      }
      if(enableClimber){
        moveClimber();
      }
    }

    // This method will be called once per scheduler run
  }
}
