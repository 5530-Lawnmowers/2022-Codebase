// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.helpers.ShuffleboardHelpers;
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
  if(climb.getAngle() < .5){
    if(climb.getAngle()<.25) {
      positionToRun = climb.getPosition(90, false);
    }
    else{
      positionToRun = climb.getPosition(90, true);

    }

  }
  else{
    if(climb.getAngle() >.75) {
      positionToRun = climb.getPosition(270, true);
    }
    else{
      positionToRun = climb.getPosition(270, false);

    }

  }
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
//    return climb.getEncoderPosition() + Constants.goodEnough > positionToRun || climb.getEncoderPosition() - Constants.goodEnough < positionToRun;
  }
}
