// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.OperatorIntake;
import frc.robot.commands.driveForward;
import frc.robot.subsystems.*;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


    public static XboxController XBController1 = new XboxController(0);
    public static XboxController XBController2 = new XboxController(1);
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Climb climb = new Climb();
    private final Shooter shooter = new Shooter();
    public static JoystickButton xb1a = new JoystickButton(XBController1, 1);
    private final Intake intake = new Intake();
    //Jeremy's Comment

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings

        configureButtonBindings();
  }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        xb1a.toggleWhenPressed(new OperatorIntake(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

      // Create a voltage constraint to ensure we don't accelerate too fast
      var autoVoltageConstraint =
              new DifferentialDriveVoltageConstraint(
                      new SimpleMotorFeedforward(
                              Constants.kS,
                              Constants.kV,
                              Constants.kA),
                      Constants.DIFFERENTIAL_DRIVE_KINEMATICS,
                      10);

      // Create config for trajectory
      TrajectoryConfig config =
              new TrajectoryConfig(
                      Constants.kMaxSpeedMetersPerSecond,
                      Constants.kMaxAccelerationMetersPerSecondSquared)
                      // Add kinematics to ensure max speed is actually obeyed
                      .setKinematics(Constants.DIFFERENTIAL_DRIVE_KINEMATICS)
                      // Apply the voltage constraint
                      .addConstraint(autoVoltageConstraint);

      // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
              TrajectoryGenerator.generateTrajectory(
                      // Start at the origin facing the +X direction
                      new Pose2d(0, 0, new Rotation2d(0)),
                      // Pass through these two interior waypoints, making an 's' curve path
                      List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                      // End 3 meters straight ahead of where we started, facing forward
                      new Pose2d(3, 0, new Rotation2d(0)),
                      // Pass config
                      config);

      RamseteCommand ramseteCommand =
              new RamseteCommand(
                      exampleTrajectory,
                      drivetrain::getPose,
                      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                      new SimpleMotorFeedforward(
                              Constants.kS,
                              Constants.kV,
                              Constants.kA),
                      Constants.DIFFERENTIAL_DRIVE_KINEMATICS,
                      drivetrain::getWheelSpeeds,
                      new PIDController(Constants.kP, 0, 0),
                      new PIDController(Constants.kP, 0, 0),
                      // RamseteCommand passes volts to the callback
                      drivetrain::tankDriveVolts,
                      drivetrain);

      // Reset odometry to the starting pose of the trajectory.
      drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return new driveForward(drivetrain, intake);
//      return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts((double) 0, (double) 0));
  }
    // An ExampleCommand will run in autonomous
    // }
}
