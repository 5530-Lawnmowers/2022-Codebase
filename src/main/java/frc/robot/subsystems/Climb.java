// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.OperatorClimb;
import frc.robot.helpers.ShuffleboardHelpers;

public class Climb extends SubsystemBase {
  private CANSparkMax ClimbL = new CANSparkMax(Constants.ClimbL, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax ClimbR = new CANSparkMax(Constants.ClimbR, CANSparkMaxLowLevel.MotorType.kBrushless);
  private SparkMaxPIDController controllerL;
  /**
   * Creates a new Climb.
   */
  private double F= 0;
  private double P= 0;
  private double I= 0;
  private double D= 0;
  private double ticksPerRev = 2048; //Find this out
  private RelativeEncoder encoder;
  public Climb() {
    encoder = ClimbL.getEncoder();
    encoder.setPosition(0);
    controllerL = ClimbL.getPIDController();
    controllerL.setFF(F, 0);
    controllerL.setP(P, 0);
    controllerL.setI(I, 0);
    controllerL.setD(D, 0);
    ShuffleboardHelpers.setWidgetValue("Climb","F",F);
    ShuffleboardHelpers.setWidgetValue("Climb","P",P);
    ShuffleboardHelpers.setWidgetValue("Climb","I",I);
    ShuffleboardHelpers.setWidgetValue("Climb","D",D);
    controllerL.setOutputRange(-.5, .5);
    ClimbL.setIdleMode(CANSparkMax.IdleMode.kBrake);
    ClimbR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    setDefaultCommand(new OperatorClimb(this));
  }


  @Override
  public void periodic() {

    //Delete this once we get it tuned
    F = (double) ShuffleboardHelpers.getWidgetValue("Climb", "F");
    P = (double) ShuffleboardHelpers.getWidgetValue("Climb", "P");
    I = (double) ShuffleboardHelpers.getWidgetValue("Climb", "I");
    D = (double) ShuffleboardHelpers.getWidgetValue("Climb", "D");
    controllerL.setFF(F, 0);
    controllerL.setP(P, 0);
    controllerL.setI(I, 0);
    controllerL.setD(D, 0);
    // This method will be called once per scheduler run
    ShuffleboardHelpers.setWidgetValue("Climb", "Current Position", getEncoderPosition());


  }

  public void setRawPower(double Power) {
    ClimbR.set(-Power);
    ClimbL.set(Power);

  }

  public void zeroPosition(){
    encoder.setPosition(0);
  }

  public double getPosition(double angle, boolean clockwise){
    double realABS = getEncoderPosition();
    double realRel =Math.abs( (realABS % ticksPerRev) / ticksPerRev);
    double modifier = (int)realABS / (int)ticksPerRev;
    if(modifier <= -1){
      realRel = 1-realRel;
    }
    double desireAngle = angle;
    double desPosRel = (desireAngle/360.0);
    double difference = desPosRel - realRel;
    double setpoint = 0;
    if(difference > 0 && !clockwise) {
      setpoint = (realRel + difference + modifier) * ticksPerRev;
    }
    else if (difference > 0 && clockwise){
      modifier--;
      setpoint = (realRel + difference + modifier) * ticksPerRev;

    }
    if(difference < 0 && clockwise){
      setpoint = (realRel + difference + modifier) * ticksPerRev;

    }
    else if (difference < 0 && !clockwise){
      modifier++;
      setpoint = (realRel + difference + modifier) * ticksPerRev;

    }
    return setpoint;

  }

  //Make sure it gives good units for closed loop

  public double getEncoderPosition(){
    return encoder.getPosition();
  }

  public void runToPosition(double position){
    controllerL.setReference(position, CANSparkMax.ControlType.kPosition);
    ClimbR.set(-ClimbL.get());
  }


}
