/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.CurvatureDriveNew;
import frc.robot.helpers.ShuffleboardHelpers;

//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import frc.robot.helpers.ShuffleboardHelpers;

public class Drivetrain extends SubsystemBase {
    private final DifferentialDriveOdometry m_odometry;
    //Drive test
    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    private Field2d field = new Field2d();
    private final float TickPerRev = 0;
    private double ticks = 2048.0 * 3.37396572727;
    private final double driveMultiplier = 0.9;
    private final float WheelRadius = 0;
    private final WPI_TalonFX drivetrainLeft1 = new WPI_TalonFX(Constants.DT_L1);
    private final WPI_TalonFX drivetrainLeft2 = new WPI_TalonFX(Constants.DT_L2);
    private final WPI_TalonFX drivetrainRight1 = new WPI_TalonFX(Constants.DT_R1);
    private final WPI_TalonFX drivetrainRight2 = new WPI_TalonFX(Constants.DT_R2);
    private final Rev2mDistanceSensor distOnboard = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);

    //Drive test
//    private final SpeedControllerGroup drivetrainLeft;
//    private final SpeedControllerGroup drivetrainRight;
//    private final DifferentialDrive diffDrive;

    public static double StartingPose;

    public static float WheelCircumference = (float) .160;//in m
    // public static DifferentialDriveOdometry DDO = new DifferentialDriveOdometry(new Rotation2d());
    //public static PigeonIMU pigeon = new PigeonIMU(15);
    // public static Rotation2d heading = new Rotation2d();
    public static float leftDistance = 0;
    public static float rightDistance = 0;
    // private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    /**
     * Creates a new Drivetrain.
     */
    public Drivetrain() {

        drivetrainLeft1.configFactoryDefault();
        drivetrainRight1.configFactoryDefault();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.velocityMeasurementWindow =32;
        config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;

        drivetrainLeft1.configAllSettings(config);
        drivetrainLeft2.configAllSettings(config);
        drivetrainRight1.configAllSettings(config);
        drivetrainRight2.configAllSettings(config);
        drivetrainLeft1.enableVoltageCompensation(false);
        drivetrainRight1.enableVoltageCompensation(false);
        drivetrainLeft2.enableVoltageCompensation(false);
        drivetrainRight2.enableVoltageCompensation(false);


        drivetrainLeft1.setInverted(false);

        drivetrainLeft2.setInverted(false);
        drivetrainRight1.setInverted(false);

        drivetrainRight2.setInverted(false);

        drivetrainLeft1.setNeutralMode(NeutralMode.Brake);
        drivetrainLeft2.setNeutralMode(NeutralMode.Brake);
        drivetrainRight1.setNeutralMode(NeutralMode.Brake);
        drivetrainRight2.setNeutralMode(NeutralMode.Brake); //ID 7 was weird TODO
        drivetrainLeft1.selectProfileSlot(0, 0);
        drivetrainRight1.selectProfileSlot(0, 0);

//        drivetrainLeft1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
//        drivetrainRight1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        drivetrainLeft1.setSelectedSensorPosition(0);
        drivetrainRight1.setSelectedSensorPosition(0);

        drivetrainRight1.config_kF(0, .0, 10);
        drivetrainRight1.config_kP(0, .05, 10);
        drivetrainRight1.config_kI(0, 0, 10);
        drivetrainRight1.config_kD(0, 0, 10);

        drivetrainLeft1.config_kF(0, .0, 10);
        drivetrainLeft1.config_kP(0, .05, 10);
        drivetrainLeft1.config_kI(0, 0, 10);
        drivetrainLeft1.config_kD(0, 0, 10);

        drivetrainRight2.config_kF(0, .0, 10);
        drivetrainRight2.config_kP(0, .05, 10);
        drivetrainRight2.config_kI(0, 0, 10);
        drivetrainRight2.config_kD(0, 0, 10);

        drivetrainLeft2.config_kF(0, .0, 10);
        drivetrainLeft2.config_kP(0, .05, 10);
        drivetrainLeft2.config_kI(0, 0, 10);
        drivetrainLeft2.config_kD(0, 0, 10);

//        drivetrainLeft = new SpeedControllerGroup(drivetrainLeft1, drivetrainLeft2);
//        drivetrainRight = new SpeedControllerGroup(drivetrainRight1, drivetrainRight2);
//        diffDrive = new DifferentialDrive(drivetrainLeft, drivetrainRight);
        // gyro.zeroYaw();
//        setDefaultCommand(new ThrottleMotorTest(this)); //Use this for motor tests
        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        setDefaultCommand(new CurvatureDriveNew(this));
//        setDefaultCommand(new NewDrive(this));
//        SmartDashboard.putData("Field", field);
        distOnboard.setEnabled(true);
        distOnboard.setAutomaticMode(true);
        distOnboard.setRangeProfile(Rev2mDistanceSensor.RangeProfile.kLongRange);
    }
    public double getDistance(){
        distOnboard.setEnabled(true);
        distOnboard.setAutomaticMode(true);
        return distOnboard.getRange(Rev2mDistanceSensor.Unit.kMillimeters) /1000;
    }
    @Override
    public void periodic() {
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Distance", getDistance());

//        field.setRobotPose(getPose());

//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Left Position", drivetrainLeft2.getMotorOutputVoltage());
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Right Position", drivetrainRight2.getMotorOutputVoltage());
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Left Velocity", drivetrainLeft2.get());
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Right Velocity", drivetrainRight2.get());
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Heading", getHeading());
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Displacement X", getPose().getTranslation().getX());
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Displacement Y", getPose().getTranslation().getY());


//        m_odometry.update(gyro.getRotation2d(), getLeftMeters(), getRightMeters());

        // This method will be called once per scheduler run
        // ShuffleboardHelpers.setWidgetValue("Drivetrain", "Left Encoder", drivetrainLeft1.getSelectedSensorPosition());
        //
        //  ShuffleboardHelpers.setWidgetValue("Drivetrain", "Right Encoder", drivetrainRight1.getSelectedSensorPosition());
    }

    /**
     * Sers the neutral mode for all drivetrain motors
     * @param brake {@code true} to set drivetrain to brake, {@code false} to set drivetrain to coast
     */
    public void setBrakeMode(boolean brake) {
        if (brake) {
            drivetrainLeft1.setNeutralMode(NeutralMode.Coast);
            drivetrainLeft2.setNeutralMode(NeutralMode.Brake);
            drivetrainRight1.setNeutralMode(NeutralMode.Coast);
            drivetrainRight2.setNeutralMode(NeutralMode.Brake);
        } else {
            drivetrainLeft1.setNeutralMode(NeutralMode.Coast);
            drivetrainLeft2.setNeutralMode(NeutralMode.Coast);
            drivetrainRight1.setNeutralMode(NeutralMode.Coast);
            drivetrainRight1.setNeutralMode(NeutralMode.Coast);
        }
    }

    //Drive test only
//    public void testDrive(double throttle, double turn) {
//        if (Math.abs(throttle) > 1)
//            throttle = Math.abs(throttle) / throttle; // if the value given was too high, set it to the max
//        throttle *= driveMultiplier; // scale down the speed
//
//
//        if (Math.abs(turn) > 1)
//            turn = Math.abs(turn) / turn; // if the value given was too high, set it to the max
//        turn *= driveMultiplier; // scale down the speed
//
//        diffDrive.arcadeDrive(throttle, turn); // function provided by the  controls y and turn speed at the same time.
//    }
//
////    Drive test only
//    public void testDriveStop() {
//        drivetrainLeft.stopMotor();
//        drivetrainRight.stopMotor();
//    }

    private double getThrottle() {
        double n = RobotContainer.XBController1.getRightTriggerAxis() -
                RobotContainer.XBController1.getLeftTriggerAxis();
        return Math.abs(n) < 0.1 ? 0 : n;
    }

    private double getTurn() {
        double n = RobotContainer.XBController1.getLeftX();
        return Math.abs(n) < 0.1 ? 0 : n;
    }

    /**
     * Set the speed of a drivetrain motor
     *
     * @param speed      The speed to set
     * @param controller Constants.DT_L1, DT_L2, DT_R1, DT_R2
     */
    public void setDrivetrainMotor(double speed, int controller) {
        if (controller == Constants.DT_L1) {
            drivetrainLeft1.set(0);
        } else if (controller == Constants.DT_L2) {
            drivetrainLeft2.set(speed);
        } else if (controller == Constants.DT_R1) {
            drivetrainRight1.set(0);
        } else if (controller == Constants.DT_R2) {
            drivetrainRight2.set(speed);
        }
    }

    public void setVelocity(double left, double right) {
        drivetrainRight1.set(ControlMode.Velocity, MetersToUnits(right));
        drivetrainRight2.set(ControlMode.Velocity, MetersToUnits(right));
        drivetrainLeft1.set(ControlMode.Velocity, MetersToUnits(left));
        drivetrainLeft2.set(ControlMode.Velocity, MetersToUnits(left));
    }

    public double MetersToUnits(double meters) {
        meters = Units.metersToInches(meters);
        meters = meters / (WheelRadius * Math.PI);
        meters = meters * TickPerRev;
        meters = meters / 10;
        return meters;
    }

    public void setPosition(double leftPos, double rightPos) {
        drivetrainLeft1.set(ControlMode.Position, leftPos);
        drivetrainLeft2.follow(drivetrainLeft1);
        drivetrainRight1.set(ControlMode.Position, rightPos);
        drivetrainRight2.follow(drivetrainRight1);
    }


    /**
     * Stops all drivetrain motors
     */
    public void testDrivetrainStop() {
        drivetrainLeft1.stopMotor();
        drivetrainLeft2.stopMotor();
        drivetrainRight1.stopMotor();
        drivetrainRight2.stopMotor();
    }

    // public int getLeftEncoder() {
    //     return (int)drivetrainLeft1.getSelectedSensorPosition();
    // }

    // public int getRightEncoder() {
    //     return (int)drivetrainRight1.getSelectedSensorPosition();
    // }
    public Pose2d getPose() {
        m_odometry.update(

                gyro.getRotation2d(), getLeftMeters(), getRightMeters());

        return m_odometry.getPoseMeters();
    }

    public double getLeftMeters() {
        double encoderPos = drivetrainLeft1.getSelectedSensorPosition();
//        ShuffleboardHelpers.setWidgetValue("Drivetrain" , "ticks", encoderPos);

        double rotations = encoderPos / ticks; //2048 ticks per rotation
        double outPutRotations = rotations * (40.0 / 52);// geared to 40:52
        double meters = outPutRotations * Units.inchesToMeters(4) * Math.PI; //Circumfraance * pi * diameter // Distance equals rotations times circumfrance
        return meters;
    }

    public double getRightMeters() {
        double encoderPos = drivetrainRight1.getSelectedSensorPosition();
        double rotations = encoderPos / ticks; //2048 ticks per rotation
        double outPutRotations = rotations * (40.0 / 52);// geared to 40:52
        double meters = outPutRotations * Units.inchesToMeters(4) * Math.PI; //Circumfraance * pi * diameter // Distance equals rotations times circumfrance
        return -meters;
    }

    public double getLeftVelocity() {
        double encoderVel = drivetrainLeft1.getSelectedSensorVelocity();
        double rotations = encoderVel / ticks; //2048 ticks per rotation
        double outPutRotations = rotations * (40.0 / 52);// geared to 40:52
        double meters = outPutRotations * Units.inchesToMeters(4) * Math.PI; //Circumfraance * pi * diameter // Distance equals rotations times circumfrance
        return meters *10 ;
    }

    public double getRightVelocity() {
        double encoderVel = drivetrainRight1.getSelectedSensorVelocity();
        double rotations = encoderVel / ticks; //2048 ticks per rotation
        double outPutRotations = rotations * (40.0 / 52);// geared to 40:52
        double meters = outPutRotations * Units.inchesToMeters(4) * Math.PI; //Circumfraance * pi * diameter // Distance equals rotations times circumfrance
        return -meters * 10;
    }

    public void resetEncoders() {
        gyro.reset();
//        gyro.resetDistplacement();
        gyro.zeroYaw();
        drivetrainLeft1.setSelectedSensorPosition(0);
        drivetrainRight1.setSelectedSensorPosition(0);
    }

    public void RESETALL(){
        drivetrainRight2.configFactoryDefault();
        drivetrainRight2.setSafetyEnabled(false);
        drivetrainRight2.enableVoltageCompensation(false);
        drivetrainLeft2.configFactoryDefault();
        drivetrainLeft2.setSafetyEnabled(false);
        drivetrainLeft2.enableVoltageCompensation(false);
    }

    public double getHeading() {
        return getPose().getRotation().getDegrees();
    }


    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        drivetrainLeft1.setVoltage(leftVolts);
        drivetrainLeft2.setVoltage(leftVolts);

        drivetrainRight1.setVoltage(-rightVolts);
        drivetrainRight2.setVoltage(-rightVolts);
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Left Volts",leftVolts);
//        ShuffleboardHelpers.setWidgetValue("Drivetrain", "Right Volts",rightVolts*.8);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, gyro.getRotation2d());
    }

}