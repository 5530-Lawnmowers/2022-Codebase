// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.helpers.ShuffleboardHelpers;
import frc.robot.subsystems.Climb;

public class climbInitalLatch extends CommandBase {
  private final Climb climb;
  private double positionToRun;

  /** Creates a new climbInitalLatch. */
  public climbInitalLatch(Climb climb) {
    addRequirements(climb);
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     positionToRun = climb.getPosition(70, true);
    ShuffleboardHelpers.setWidgetValue("Climb", "Set Position", positionToRun);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.runToPosition(positionToRun);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setRawPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(Math.abs(climb.getEncoderPosition()-positionToRun) < Constants.goodEnough) {
      return true;
    }
    else{
      return false;
    }
  }
}
