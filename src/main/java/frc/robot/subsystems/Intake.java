// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.LimelightHelper;
import frc.robot.helpers.RumbleHelp;

public class Intake extends SubsystemBase {
//  private Servo leftServo = new Servo(0);
//  private Servo rightServo = new Servo(1);
  private CANSparkMax Intake = new CANSparkMax(Constants.Intake, CANSparkMaxLowLevel.MotorType.kBrushed);
  Servo leftServo = new Servo(0);
  Servo rightServo = new Servo(1);

  /**
   * Creates a new Intake.
   */
  public Intake() {
    leftServo.set(1);
    rightServo.set(0);
    Intake.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }

  public void setServo(){
    leftServo.set(1);
    rightServo.set(0);
  }
  public void Release(){
    rightServo.set(1);
    leftServo.set(0);
  }
  public double position(){
    return leftServo.get();
  }
  public boolean Released(){
    return rightServo.get() > .75;
  }
  @Override
  public void periodic() {

//    if(LimelightHelper.getFound()){
//      RumbleHelp.RumbleStart();
//    }
//    else{
//      RumbleHelp.RumbleStop();
//    }
    // This method will be called once per scheduler run
  }

  public void setRawPower(double power) {
    Intake.set(power);
  }
}
