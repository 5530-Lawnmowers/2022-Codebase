/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class CurvatureDriveNew extends CommandBase {
  private final double quickstopAlpha = 0.1;
  private final double quickstopThreshold = 0.2;
  private final double turnWeight = 0.75;
  private final double driveWeight = 0.85;
  private final double kDeadbandTrigger = 0.02;
  private final double kDeadbandJoystick = 0.2;

  private double oldTriggerL;
  private double oldTriggerR;
  private double quickstopAccumulator = 0;

  private Drivetrain drivetrain;

  /**
   * Creates a new CurvatureDrive.
   */
  public CurvatureDriveNew(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set initial trigger values
    oldTriggerL = deadband(RobotContainer.XBController1.getLeftTriggerAxis(), kDeadbandTrigger);
    oldTriggerR = deadband(RobotContainer.XBController1.getRightTriggerAxis(), kDeadbandTrigger);

    // Drivetrain to coast
    drivetrain.setBrakeMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // Get preliminary current triggers
    double accelerator = deadband(RobotContainer.XBController1.getRightTriggerAxis(), kDeadbandTrigger);
    double decelerator = deadband(RobotContainer.XBController1.getLeftTriggerAxis(), kDeadbandTrigger);

    // Ramp up input values as needed
    if (Math.abs(accelerator - oldTriggerR) > 0.2) {
      accelerator = ramp(accelerator, "right");
    }

    if (Math.abs(decelerator - oldTriggerL) > 0.2) {
      decelerator = ramp(decelerator, "left");
    }

    // Define necessary inputs SWITCH TO DRIVE BACK
    double throttle = decelerator - accelerator;
    double curve = deadband(RobotContainer.XBController1.getLeftX(), kDeadbandJoystick);
    double quickTurn = deadband(RobotContainer.XBController1.getRightX(), kDeadbandJoystick) * turnWeight;

    if (Math.abs(throttle) < quickstopThreshold) {
      // quickstopAccumulator to account for inertia
      quickstopAccumulator = (1 - quickstopAlpha) * quickstopAccumulator + quickstopAlpha * limit(quickTurn, 1) * 2;
    }

    // Set left and right power from quickTurn
    double leftPower = throttle + quickTurn;
    double rightPower = throttle - quickTurn;

    // Account for overpowering rotations
    if (leftPower > 1) {
      rightPower -= leftPower - 1;
      leftPower = 1;
    } else if (leftPower < -1) {
      rightPower -= leftPower + 1;
      leftPower = -1;
    } else if (rightPower > 1) {
      leftPower -= rightPower - 1;
      rightPower = 1;
    } else if (rightPower < -1) {
      leftPower -= rightPower + 1;
      rightPower = -1;
    }

    // Find curvePower from curvature
    double curvePower = Math.abs(throttle) * curve - quickstopAccumulator;

    // Apply and adjust quickstopAccumulator to account for inertia
    if (quickstopAccumulator > 1) {
      quickstopAccumulator -= 1;
    } else if (quickstopAccumulator < -1) {
      quickstopAccumulator += 1;
    } else {
      quickstopAccumulator = 0;
    }

    // Apply curve to quickTurn power
    leftPower += curvePower;
    rightPower -= curvePower;

    // Normalize inputs
    double maxInput = Math.max(Math.abs(leftPower), Math.abs(rightPower));
    if (maxInput > 1) {
      leftPower /= maxInput;
      rightPower /= maxInput;
    }

    // Feed to motors
    System.out.println(leftPower);
    drivetrain.setDrivetrainMotor(driveWeight * leftPower, Constants.DT_L1);
    drivetrain.setDrivetrainMotor(driveWeight * leftPower, Constants.DT_L2);
    drivetrain.setDrivetrainMotor(-driveWeight * rightPower, Constants.DT_R1);
    drivetrain.setDrivetrainMotor(-driveWeight * rightPower, Constants.DT_R2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Clamps the given value to within a given magnitude.
   * 
   * @param value        the value to clamp
   * @param maxMagnitude the maximum magnitude to return
   * @return the clamped value
   */
  private double limit(double value, double maxMagnitude) {
    if (Math.abs(value) < maxMagnitude) {
      return value;
    } else {
      return Math.signum(value) * maxMagnitude;
    }
  }

  /**
   * Takes care of deadbanding
   * 
   * @param input     the input value
   * @param threshold the deadband value
   * @return {@code 0} if input is within deadband, {@code input} otherwise
   */
  private double deadband(double input, double threshold) {
    if (Math.abs(input) > threshold) {
      return input;
    } else {
      return 0;
    }
  }

  /**
   * Ramps up trigger input based on previously stored value
   *
   * @param newTrigger the new input value
   * @param hand       the side input is from
   * @return the ramped input value
   */
  public double ramp(double newTrigger, String hand) {
    if (hand.equals("left")) {
      oldTriggerL = 0.09516 * newTrigger + 0.9048 * oldTriggerL;
      return oldTriggerL;
    } else if (hand.equals("right")) {
      oldTriggerR = 0.09516 * newTrigger + 0.9048 * oldTriggerR;
      return oldTriggerR;
    } else {
      return 0;
    }
  }
}
