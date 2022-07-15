// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Drivetrain motors
    public static final int DT_L1 = 5;
    public static final int DT_L2 = 6;
    public static final int DT_R1 = 7;
    public static final int DT_R2 = 8;


    public static final int index =62;


    //Intake Motors
    public static final int Intake = 11;
    //the big one



    public  static final int Intake2 = 17;//Not Used
    public  static final int turret = 52; //br drive


    //shooter motors
    public static final int ShootTop = 10;
    public static final int ShootBot = 9;
    public static final int hood = 19;

    //climb motors
    public static final int ClimbL = 2;
    public static final int ClimbR = 3;

    //Climb Constants

    public static final double goodEnough =  2;

    //Drivetrain Constants
    public static final double kS = 0.72666;
    public static final double kV = 0.88984;
    public static final double kA = 0.18986;
    public static final double kP = 1.3132;
    public static final double kTrackwidthMeters = 0.5588;
    public static final DifferentialDriveKinematics DIFFERENTIAL_DRIVE_KINEMATICS = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;//6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;// 3;
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;


}
