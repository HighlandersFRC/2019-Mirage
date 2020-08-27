/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class RobotStats {
    public static final double encoderTicksPerWheelRotation = 11170;
    public static final double wheelDiam = 6;
    public static final double wheelCircum = wheelDiam * Math.PI;
    public static double ticksToInches(double ticks) {
        return (ticks / encoderTicksPerWheelRotation) * wheelCircum;
    }
    public static double inchesToTicks(double inches) {
       return (inches / wheelCircum) * encoderTicksPerWheelRotation;

    }
}
