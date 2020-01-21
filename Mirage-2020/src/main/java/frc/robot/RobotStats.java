/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class RobotStats {
    public static final double robotBaseDistance = 2.1;
    public static final double robotMaxAccertion= 12;
    public static final double robotMaxVelocity = 12;
    public static final double encoderTicsPerWheelRotation = 11170;
    public static final double wheelDiam =6;
    public static final double wheelCircum = (wheelDiam*Math.PI)/12;
    public static final double joyStickDeadZone = 0.015;
    public static final double triggerDeadZone = 0.1;
    public static final int driveMotorContinuousCurrentHighGear = 30;
    public static final int driveMotorPeakCurrentHighGear = 40;
    public static final int intakePeakCurrent = 40;
    public static final int intakeContinuousCurrent = 20;
    public static final int driveMotorPeakCurrentDurationHighGear = 50;
}
