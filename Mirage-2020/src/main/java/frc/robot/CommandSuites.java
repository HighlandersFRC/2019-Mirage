/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.tools.controlLoops.PurePursuitController;

/**
 * Add your docs here.
 */
public class CommandSuites {
    private PurePursuitController purePursuitController;
    
    public CommandSuites(){
    }
    public void startAutoCommands(){
        purePursuitController  = new PurePursuitController(RobotMap.pathlist.path1, 1.0, 2.0, true, false);
        purePursuitController.start();

    }
    public void endAutoCommands(){

    }
    public void startTeleopCommands(){
    }
    public void endTeleopCommands(){

    }
}
