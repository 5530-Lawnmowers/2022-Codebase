// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;


/**
 * Add your docs here.
 */


public class RumbleHelp {


    public static void RumbleStart(){
            RobotContainer.XBController1.setRumble(RumbleType.kLeftRumble, .25);
            RobotContainer.XBController1.setRumble(RumbleType.kRightRumble, .25);
    }
    public static void RumbleStop(){
            RobotContainer.XBController1.setRumble(RumbleType.kLeftRumble, 0);
            RobotContainer.XBController1.setRumble(RumbleType.kRightRumble, 0);
            RobotContainer.XBController2.setRumble(RumbleType.kLeftRumble, 0);
            RobotContainer.XBController2.setRumble(RumbleType.kRightRumble, 0);
    }
    public static void RumbleReady(){
            RobotContainer.XBController2.setRumble(RumbleType.kLeftRumble, .75);
            RobotContainer.XBController2.setRumble(RumbleType.kRightRumble, .75);
    }
    public static void RumbleClimb(){
            RobotContainer.XBController1.setRumble(RumbleType.kLeftRumble, .75);
            RobotContainer.XBController1.setRumble(RumbleType.kRightRumble, .75);
            RobotContainer.XBController2.setRumble(RumbleType.kLeftRumble, .75);
            RobotContainer.XBController2.setRumble(RumbleType.kRightRumble, .75);
    }
}
