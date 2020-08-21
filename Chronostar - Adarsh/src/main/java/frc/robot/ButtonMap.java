package frc.robot;

import frc.robot.OI;

public class ButtonMap {
    public static OI oi = new OI();

    public static double getRightSide() {
        return oi.driverController.getRawAxis(5);
    }

    public static double getLeftSide() {
        return oi.driverController.getRawAxis(1);
    }

    public static double getForwardMovement() {
        return oi.driverController.getRawAxis(1);
    }

    public static double getTurningMovement() {
        return oi.driverController.getRawAxis(4);
    }
    public static boolean xButtonPressed(){
        return oi.driverController.getXButtonPressed();
    }
    public static boolean xButtonReleased(){
        return oi.driverController.getXButtonReleased();
    }

    public static boolean aButtonPressed(){
        return oi.driverController.getAButtonPressed();
    }
    public static boolean aButtonReleased(){
        return oi.driverController.getAButtonReleased();
    }
}