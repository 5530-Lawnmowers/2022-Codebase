// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class driveForward extends CommandBase {
  private final Drivetrain drivertrain;
  private final Intake intake;
  private Timer timer = new Timer();

  /**
   * Creates a new driveForward.
   */
  public driveForward(Drivetrain drivetrain, Intake intake) {
    addRequirements(drivetrain, intake);
    this.drivertrain = drivetrain;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setRawPower(-.8);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivertrain.setPosition(-50000, 50000);
    if (timer.advanceIfElapsed(5)) {
      intake.setRawPower(0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRawPower(0);

    drivertrain.testDrivetrainStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
