// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.ShuffleboardHelpers;
import frc.robot.subsystems.Climb;

public class ClimbToAngleFast extends CommandBase {
  private Climb climb;
  private double angle;
  private boolean clockwise;
  private boolean weGoingForward;
private boolean weGoodToStop = false;
  /** Creates a new ClimbToAngleFast. */
  private double positionToGoTo;
  public ClimbToAngleFast(Climb climb, double angle, boolean clockwise) {
    addRequirements(climb);
    this.climb = climb;
    this.angle = angle;
    this.clockwise = clockwise;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    positionToGoTo = climb.getPosition(angle,clockwise);
    ShuffleboardHelpers.setWidgetValue("Climb", "Set Position", positionToGoTo);
    ShuffleboardHelpers.setWidgetValue("Climb","ran",false);
    ShuffleboardHelpers.setWidgetValue("Climb","We Good To Stop",false);
    ShuffleboardHelpers.setWidgetValue("Climb","We Going Forward",false);

    double difference = climb.getEncoderPosition() - positionToGoTo;
    if(difference < 0){
      weGoingForward = true;
    }
    else if (difference == 0){
      weGoodToStop = true;
    }
    else{
      weGoingForward = false;
    }
    weGoodToStop=false;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(weGoingForward && (climb.getEncoderPosition() > positionToGoTo) ){
      climb.setRawPower(0);
      weGoodToStop = true;
      ShuffleboardHelpers.setWidgetValue("Climb","ran",true);
    }
    else if(weGoingForward){
      climb.setRawPower(.5);

    }
    else if (!weGoingForward && (climb.getEncoderPosition() < positionToGoTo)){
      climb.setRawPower(0);
      weGoodToStop = true;
      ShuffleboardHelpers.setWidgetValue("Climb","ran",true);


    }
    else if(!weGoingForward){
      climb.setRawPower(-.5);
    }
    ShuffleboardHelpers.setWidgetValue("Climb","We Good To Stop",weGoodToStop);

    ShuffleboardHelpers.setWidgetValue("Climb","We Going Forward",weGoingForward);




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setRawPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return weGoodToStop;
  }
}
