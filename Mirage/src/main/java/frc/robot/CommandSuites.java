/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.commands.humanInterface.DriveInterface;
import frc.robot.tools.controlLoops.VelocityPID;

/**
 * Add your docs here.
 */
public class CommandSuites {
    private DriveInterface driveInterface;
    public CommandSuites(){
        driveInterface = new DriveInterface();
    }
    public void startAutoCommands(){
    }
    public void endAutoCommands(){

    }
    public void startTeleopCommands(){
        driveInterface.start();
    }
    public void endTeleopCommands(){

    }
}
