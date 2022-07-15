// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class pickupBall extends CommandBase {
  private final Drivetrain drivetrain;

  /** Creates a new pickupBall. */
  public pickupBall(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(drivetrain.getDistance() < 0  || drivetrain.getDistance() <.6){
      setLeftPower(0);
      setRightPower(0);

    }
    else{
      setLeftPower(-.2);
      setRightPower(-.2);

    }

  }
  void setLeftPower(double leftPower){
    drivetrain.setDrivetrainMotor(leftPower ,5);
    drivetrain.setDrivetrainMotor(leftPower,6);

  }
  void setRightPower(double rightPower){
    drivetrain.setDrivetrainMotor(-rightPower,7);
    drivetrain.setDrivetrainMotor(-rightPower,8);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setLeftPower(0);
    setRightPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivetrain.getDistance() < 0  || drivetrain.getDistance() <.6){
      return true;
    }
    return false;
  }
}
