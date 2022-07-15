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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.lang.reflect.Field;
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
//    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Climb climb = new Climb();
    private final Shooter shooter = new Shooter();
//    private final Turret turret = new Turret();
private final Index index = new Index();
//    public static JoystickButton xb1leftStick = new JoystickButton(XBController1, 9);
public static JoystickButton xb1a = new JoystickButton(XBController1, 1);
////    public static JoystickButton xb1b = new JoystickButton(XBController1, 2);
//    public static JoystickButton xb1y = new JoystickButton(XBController1, 4);
//    public static JoystickButton xb1x = new JoystickButton(XBController1, 3);
////    public static JoystickButton xb1lb = new JoystickButton(XBController1, 5);
//    public static JoystickButton xb1rb = new JoystickButton(XBController1, 6);
//    public static JoystickButton xbstart = new JoystickButton(XBController1, 8);


//    public static JoystickButton xb2a = new JoystickButton(XBController2, 1);
//    public static JoystickButton xb2b = new JoystickButton(XBController2, 2);
//    public static JoystickButton xb2lb = new JoystickButton(XBController2, 5);
//    public static JoystickButton xb2rb = new JoystickButton(XBController2, 6);
//    public static JoystickButton xb2y = new JoystickButton(XBController2, 4);
//    public static JoystickButton xb2x = new JoystickButton(XBController2, 3);
//    public static JoystickButton xb2start = new JoystickButton(XBController2, 8);
//    public static JoystickButton xb2back = new JoystickButton(XBController2, 7);
//    public static JoystickButton xb2lstick = new JoystickButton(XBController2, 9);
//    public static JoystickButton xb2rstick = new JoystickButton(XBController2, 10);



    private final Intake intake = new Intake();
    //Jeremy's Comment
    private final Hood hood = new Hood();

//    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        command ->
                                Shuffleboard.addEventMarker(
                                        "Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        command ->
                                Shuffleboard.addEventMarker(
                                        "Command interrupted", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandFinish(
                        command ->
                                Shuffleboard.addEventMarker(
                                        "Command finished", command.getName(), EventImportance.kNormal));
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
        //xb2a.whenHeld(new climbReadyLatch(climb));
        //xb2b.whenHeld(new climbInitalLatch(climb));
        //xb2x.whenHeld(new ClimbToAngleFast(climb,90,false));
//        xb1leftStick.whenHeld(new alignRobot(drivetrain, false));
//        xb1x.whenHeld(new alignAngle(drivetrain,false));
//        xb1y.whenHeld(new pickupBall(drivetrain));
//        xb2a.toggleWhenPressed(new OperatorIntake(intake));
//
//        xb2b.whenHeld(new IndexOperator(index));
//        xb2y.whenHeld(new IndexOperatorOut(index));
//        xb2x.whenHeld(new IntakeOut(intake));
//        xb2lb.toggleWhenPressed(new ShooterTesting(shooter));
//        xb2rb.toggleWhenPressed(new controlHood(hood));
//        xb2start.whenPressed(new dropIntake(intake));
//                xb2lb.whenHeld(new preset_Auton(shooter,hood));
//        xb1rb.whenHeld(new alignLimelight(hood));
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
                      .addConstraint(autoVoltageConstraint)
                      .setReversed(true);

      // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
              TrajectoryGenerator.generateTrajectory(
                      // Start at the origin facing the +X direction
                      new Pose2d(0, 0, new Rotation2d(0)),
                      // Pass through these two interior waypoints, making an 's' curve path
//                      List.of(new Translation2d(3, -0.747314)),
//                                            List.of(new Translation2d(-3, -1.993)),
                      List.of(new Translation2d(-2.5, -1.993)),


                      // End 3 meters straight ahead of where we started, facing forward
//                      new Pose2d(3.991212, -0.747314, new Rotation2d(0)),
//                      new Pose2d(-3.537, -1.993, new Rotation2d(0)),
                      new Pose2d(-3.0, -1.993, new Rotation2d(0)),

                      // Pass config
                      config);

      Trajectory meter =
              TrajectoryGenerator.generateTrajectory(
                      // Start at the origin facing the +X direction
                      new Pose2d(0, 0, new Rotation2d(0)),
                      // Pass through these two interior waypoints, making an 's' curve path
//                      List.of(new Translation2d(3, -0.747314)),
                      List.of(new Translation2d(-.5, 0)),

                      // End 3 meters straight ahead of where we started, facing forward
//                      new Pose2d(3.991212, -0.747314, new Rotation2d(0)),
                      new Pose2d((-1), 0, new Rotation2d(0)),

                      // Pass config
                      config);

      Field2d field = new Field2d();
SmartDashboard.putData(field);
    field.getObject("traj").setTrajectory(meter);

      RamseteCommand ramseteCommand =
              new RamseteCommand(
                      meter,
                      drivetrain::getPose,
                      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                      new SimpleMotorFeedforward(
                              Constants.kS,
                              Constants.kV,
                              Constants.kA),
                      Constants.DIFFERENTIAL_DRIVE_KINEMATICS,
                      drivetrain::getWheelSpeeds,
                      new PIDController(Constants.kP, 0, 0), //left controller
                      new PIDController(Constants.kP , 0, 0), //right controller

                      // RamseteCommand passes volts to the callback
                      drivetrain::tankDriveVolts,
                      drivetrain);

      RamseteCommand ramseteCommand2electricbugolo =
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
                      new PIDController(Constants.kP, 0, 0), //left controller
                      new PIDController(Constants.kP , 0, 0), //right controller

                      // RamseteCommand passes volts to the callback
                      drivetrain::tankDriveVolts,
                      drivetrain);
//TODO put this outside the method so it runs faster

      // Reset odometry to the starting pose of the trajectory.
      drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

      // Run path following command, then stop at the end.

      //TODO electric tape terminal power off sticker
//      return new driveForward(drivetrain, intake);





//      return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts((double) 0, (double) 0));


//      return new ParallelCommandGroup(
//              new OperatorIntake(intake),
//                new SequentialCommandGroup(ramseteCommand.andThen(() -> drivetrain.tankDriveVolts((double) 0, (double) 0))
//                        ,new wait(4.0),
//                        new resetOdometer(drivetrain, meter.getInitialPose()),
//                        new wait(1),
//                        ramseteCommand2electricbugolo.andThen(() -> drivetrain.tankDriveVolts((double) 0, (double) 0))
//                        ,new wait(5)
//                        ,new alignAngle(drivetrain,true)
//                        ,new alignRobot(drivetrain,true)
//                        ,new pickupBall(drivetrain)
//                         )
//                        );

      return new ParallelCommandGroup(
                      new OperatorIntake(intake)


              ,
              new preset_Auton(shooter, hood),
              new SequentialCommandGroup(

                      ramseteCommand.andThen(() -> drivetrain.tankDriveVolts((double) 0, (double) 0)),
                      new IndexOperator(index)



              )


      );

  }

//  RamseteCommand generateRamsete(){
//  }





    // An ExampleCommand will run in autonomous
    // }
}
