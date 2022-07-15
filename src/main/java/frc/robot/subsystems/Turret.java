// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.ShuffleboardHelpers;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
//  public static CANSparkMax turretMotor = new CANSparkMax(Constants.ClimbR, CANSparkMaxLowLevel.MotorType.kBrushed);
  public double speed;
  final double minposition = 4;
  final double maxposition = 20;
//  private TalonSRX turret = new TalonSRX(Constants.turret);


  public Turret() {
//    turret.setNeutralMode(NeutralMode.Brake);

  }

  public void setPower(double speed) {
    this.speed = speed;

//    turretMotor.set(speed);
  }


  @Override
  public void periodic() {
//    ShuffleboardHelpers.setWidgetValue("Turret", "Position",  getPosition() %1025);
//    if (minposition < getPosition() && (speed < 0)) {
//      setPower(0);
//    } else if (maxposition > getPosition() && speed > 0) {
//      setPower(0);
//    } else {
//      setPower(speed);
//    }
    // This method will be called once per scheduler run


  }

  public double getPosition() {
    return 0;
  }
}

