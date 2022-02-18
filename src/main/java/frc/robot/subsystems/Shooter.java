// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterTesting;
import frc.robot.helpers.ShuffleboardHelpers;

public class Shooter extends SubsystemBase {
  private CANSparkMax Bot = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax Top = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    ShuffleboardHelpers.createSimpleWidget("Shooter", "TopPower", (double) 0);
    ShuffleboardHelpers.createSimpleWidget("Shooter", "BotPower", (double) 0);
    setDefaultCommand(new ShooterTesting(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRawTop(double Power) {
    Bot.set(Power);
  }

  public void setRawBot(double Power) {
    Top.set(Power);
  }

  public void setVelocityTop(double Velocity) {

  }

  public void setVelocityBot(double Velocity) {
    //TODO: Code and Tune PID ESTABLISHED UNITS
  }
}
