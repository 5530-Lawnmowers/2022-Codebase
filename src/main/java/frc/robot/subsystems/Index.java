// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Index extends SubsystemBase {
  /** Creates a new index. */
  private final CANSparkMax index = new CANSparkMax(Constants.index, CANSparkMaxLowLevel.MotorType.kBrushed);
  public Index() {

  }

  public void TurnForward(){
    index.set(1);
  }

  public void TurnBackwards(){
    index.set(-1);
  }
  public void Stop(){
    index.set(0);
  }

  public void setPower(double power){
    index.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
