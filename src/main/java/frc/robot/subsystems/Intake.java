// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax Intake = new CANSparkMax(Constants.Intake, CANSparkMaxLowLevel.MotorType.kBrushed);

  /**
   * Creates a new Intake.
   */
  public Intake() {
    Intake.setIdleMode(CANSparkMax.IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRawPower(double power) {
    Intake.set(power);
  }
}
