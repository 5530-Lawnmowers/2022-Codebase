// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.OperatorClimb;

public class Climb extends SubsystemBase {
  private CANSparkMax ClimbL = new CANSparkMax(Constants.ClimbL, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax ClimbR = new CANSparkMax(Constants.ClimbR, CANSparkMaxLowLevel.MotorType.kBrushless);

  /**
   * Creates a new Climb.
   */
  public Climb() {
    ClimbL.follow(ClimbR);
    ClimbL.setIdleMode(CANSparkMax.IdleMode.kBrake);
    ClimbR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    setDefaultCommand(new OperatorClimb(this));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRawPower(double Power) {
    ClimbR.set(Power);

  }
}
