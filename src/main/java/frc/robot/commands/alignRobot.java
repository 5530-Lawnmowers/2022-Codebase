// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.helpers.LimelightHelper;
import frc.robot.helpers.RumbleHelp;
import frc.robot.helpers.ShuffleboardHelpers;
import frc.robot.subsystems.Drivetrain;

import java.sql.Time;

public class alignRobot extends CommandBase {
  private final Drivetrain drivetrain;
    private final boolean timed;
    private PIDController pid = new PIDController(.0085,.000005,0);
    Timer time = new Timer();

  /** Creates a new alignRobot. */
  public alignRobot(Drivetrain drivetrain, boolean timed) {
      this.timed = timed;
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      time.reset();
      time.start();
      pid.setP(.01);
      pid.setI((double)ShuffleboardHelpers.getWidgetValue("Drivetrain", "Turn I"));
      pid.setD((double)ShuffleboardHelpers.getWidgetValue("Drivetrain", "Turn D"));


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double power = pid.calculate(LimelightHelper.getTurretRawX(), 0);
    if(Math.abs(LimelightHelper.getTurretRawX()) < 2 ){
        setLeftPower(0);
        setRightPower(0);
    }
    else if(power > .3){
        setLeftPower(-.3);
        setRightPower(.3);
    }
    else if(power <-.3){
        setLeftPower(.3);
        setRightPower(-.3);
    }
    else{
        setLeftPower(-power);
        setRightPower(power);
    }

    //





  }
 void setLeftPower(double leftPower){
      double controller = RobotContainer.XBController1.getRightTriggerAxis() - RobotContainer.XBController1.getLeftTriggerAxis();

   drivetrain.setDrivetrainMotor(leftPower - controller ,5);
   drivetrain.setDrivetrainMotor(leftPower - controller,6);

 }
 void setRightPower(double rightPower){
     double controller = RobotContainer.XBController1.getRightTriggerAxis() - RobotContainer.XBController1.getLeftTriggerAxis();

     drivetrain.setDrivetrainMotor(-rightPower + controller,7);
   drivetrain.setDrivetrainMotor(-rightPower + controller,8);

 }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      time.reset();
      time.stop();
    setLeftPower(0);

      setRightPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//      if(Math.abs(LimelightHelper.getTurretRawX()) < 4 ){
//          return true;
//      }
//      else if (timed && time.advanceIfElapsed(1.5))
//      {
//          return true;
//      }
      return false;
  }
}
