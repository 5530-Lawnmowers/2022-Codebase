// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.ShuffleboardHelpers;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class preset_Auton extends CommandBase {
  private final Shooter shooter;
  private final Hood hood;

  /** Creates a new preset_Auton. */
  public preset_Auton(Shooter shooter, Hood hood) {
    addRequirements(shooter, hood);
    this.shooter = shooter;
    this.hood = hood;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

//    shooter.setRawBot(-.59);
//    shooter.setRawTop(.35);
    shooter.setVoltage(-6.53,3.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRawBot(0);
    shooter.setRawTop(0);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
