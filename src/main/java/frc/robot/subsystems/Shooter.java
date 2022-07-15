// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ShooterTesting;
import frc.robot.helpers.ShuffleboardHelpers;

public class Shooter extends SubsystemBase {
  private CANSparkMax Bot = new CANSparkMax(Constants.ShootBot, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax Top = new CANSparkMax(Constants.ShootTop, CANSparkMaxLowLevel.MotorType.kBrushless);
  private SparkMaxPIDController pidBot = Bot.getPIDController();
  private SparkMaxPIDController pidTop = Top.getPIDController();
  private double P_Top = 0;
  private double I_Top = 0;
  private double D_Top = 0;

  private double P_Bot = 0;
  private double I_Bot = 0;
  private double D_Bot = 0;


  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    ShuffleboardHelpers.createSimpleWidget("Shooter", "TopPower", (double) .25);
    ShuffleboardHelpers.createSimpleWidget("Shooter", "BotPower", (double) .25);
//    setDefaultCommand(new ShooterTesting(this));

    pidBot.setP(P_Bot,0);
    pidBot.setI(I_Bot,0);
    pidBot.setD(D_Bot,0);

    pidTop.setP(P_Top,0);
    pidTop.setI(I_Top,0);
    pidTop.setD(D_Top,0);

    pidTop.setFeedbackDevice(Top.getEncoder());
    pidBot.setFeedbackDevice(Bot.getEncoder());

  }

  public double getvelocity(){
    return Bot.getEncoder().getVelocity();
  }
  public void setVoltage(double voltageTop, double voltageBot){
    Bot.setVoltage(voltageBot);
    Top.setVoltage(voltageTop);
  }

  @Override
  public void periodic() {
//    P_Bot = (double) ShuffleboardHelpers.getWidgetValue("Shooter","P_Bot");
//    I_Bot = (double) ShuffleboardHelpers.getWidgetValue("Shooter","I_Bot");
//    D_Bot = (double) ShuffleboardHelpers.getWidgetValue("Shooter","D_Bot");
//
//    P_Top = (double) ShuffleboardHelpers.getWidgetValue("Shooter","P_Top");
//    P_Top = (double) ShuffleboardHelpers.getWidgetValue("Shooter","I_Top");
//    P_Top = (double) ShuffleboardHelpers.getWidgetValue("Shooter","D_Top");
//
//
//
//
//
//    pidBot.setP(P_Bot,0);
//    pidBot.setI(I_Bot,0);
//    pidBot.setD(D_Bot,0);
//
//    pidTop.setP(P_Top,0);
//    pidTop.setI(I_Top,0);
////    pidTop.setD(D_Top,0);
//
//    ShuffleboardHelpers.setWidgetValue("Shooter", "Velocity", Bot.getEncoder().getVelocity());
//
//    ShuffleboardHelpers.setWidgetValue("Shooter", "Velocity2", Top.getEncoder().getVelocity());
//    // This method will be called once per scheduler run
  }

  public void setRawTop(double Power) {
    Bot.set(Power);
  }

  public void setRawBot(double Power) {
    Top.set(Power);
  }

  public void setVelocityTop(double Velocity) {

    pidTop.setReference(Velocity, CANSparkMax.ControlType.kVelocity);




  }

  public void setVelocityBot(double Velocity) {
    pidBot.setReference(Velocity, CANSparkMax.ControlType.kVelocity);
  }
}
