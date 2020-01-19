/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class ButtonMap {
    public static OI oi = new OI();
    public static double getDriveThrottle(){
        return -oi.driverController.getRawAxis(1);
    } 
    public static double getRotation(){
        return oi.driverController.getRawAxis(4);
    }
    public static boolean runAuto(){
        return oi.driverController.getYButton();
    }
    public static boolean shootyUp(){
        return oi.operatorController.getYButton();
    }
    public static boolean shootyDown(){
        return oi.operatorController.getAButton();
    }
    public static boolean shootyStop(){
        return oi.operatorController.getXButton();
    }
    public static double moveClimberArmUp(){
        return oi.operatorController.getRawAxis(1)*-1.0;
    }
    public static boolean enableClimberFunctions(){
        return oi.operatorController.getStartButton();
    }
}
