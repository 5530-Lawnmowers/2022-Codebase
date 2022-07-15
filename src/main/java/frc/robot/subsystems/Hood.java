// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.controlHood;
import frc.robot.helpers.LimelightHelper;

public class Hood extends SubsystemBase {
  private CANSparkMax hoodMotor = new CANSparkMax(Constants.hood, CANSparkMaxLowLevel.MotorType.kBrushed);
  /** Creates a new Hood. */
  public Hood() {
    setDefaultCommand(new controlHood(this));
  }
  public void setHood(double power){
    hoodMotor.set(power);
  }

  public void AlignHood(double offset){

    hoodMotor.set(
    LimelightHelper.getIntakeRawY() * .50
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
