// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class climbReadyLatch extends CommandBase {
  private final Climb climb;
  private double positionToRun;
  /** Creates a new climbReadyLatch. */
  public climbReadyLatch(Climb climb) {
    addRequirements(climb);
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    positionToRun = climb.getPosition(90,false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.runToPosition(positionToRun);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(climb.getEncoderPosition() +20 > positionToRun )
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
