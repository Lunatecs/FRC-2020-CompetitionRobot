/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import static frc.robot.Constants.DrivetrainConstants;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorWheelConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftFront = new WPI_TalonFX(DrivetrainConstants.Left_Front_ID);
  private final WPI_TalonFX leftBack = new WPI_TalonFX(DrivetrainConstants.Left_Back_ID);

  private final WPI_TalonFX rightFront = new WPI_TalonFX(DrivetrainConstants.Right_Front_ID);
  private final WPI_TalonFX rightBack = new WPI_TalonFX(DrivetrainConstants.Right_Back_ID);

  private static NeutralMode DRIVE_NEUTRALMODE = NeutralMode.Brake;

  private WPI_TalonSRX colorWheel = new WPI_TalonSRX(ColorWheelConstants.ColorWheel_ID);
  private PigeonIMU gyro = new PigeonIMU(colorWheel);
  private DifferentialDrive drive;
  private DifferentialDriveOdometry driveOdometry;
  /**
   * Creates a new DrivetrainSubsystem.
   */
  public DrivetrainSubsystem() {
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();

    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();

    leftFront.setNeutralMode(DRIVE_NEUTRALMODE);
    leftBack.setNeutralMode(DRIVE_NEUTRALMODE);
    
    rightFront.setNeutralMode(DRIVE_NEUTRALMODE);
    rightBack.setNeutralMode(DRIVE_NEUTRALMODE);

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    this.drive = new DifferentialDrive(leftFront, rightFront);
    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("R Encoder", this.getRightEncoderValue());
    SmartDashboard.putNumber("L Encoder", this.getLeftEncoderValue());
    SmartDashboard.putNumber("R Encoder Distance", this.getRightEncoderDistance());
    SmartDashboard.putNumber("L Encoder Distance", this.getLeftEncoderDistance());
    SmartDashboard.putNumber("Gyro Angle", this.getAngle());
    SmartDashboard.putNumber("Gyro Rate", this.getTurnRate());

    driveOdometry.update(Rotation2d.fromDegrees(getAngle()), getLeftEncoderDistance(), getRightEncoderDistance());
    // This method will be called once per scheduler run
  }

  /**
   * Applys speed and rotation to the drivetrain using the
   * arcade drive control system
   * @param speed
   * @param rotation
   */
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  /**
   * Applys speed and rotation to the drivetrain using the
   * curvature drive (Formally cheesy drive) control system
   * @param speed
   * @param rotation
   * @param isQuickTurn
   */
  public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
    drive.curvatureDrive(speed, rotation, isQuickTurn);
  }

  /**
   * Gets the left velocity in meters every second by using known
   * constants for distance per pulse of the Talon FX integrated
   * encoder. Output is multiplyed by 10 by default due to the 
   * integrated encoders default 100ms sampling time. Thus returning
   * meters every second and instead of centimeters every second. 
   * @return velocity m/s
   */
  public double getLeftEncoderDistanceRate() {
    return (leftFront.getSelectedSensorVelocity(0)* DrivetrainConstants.DistancePerPulse) * 10;
  }

    /**
   * Gets the right velocity in meters every second by using known
   * constants for distance per pulse of the Talon FX integrated
   * encoder. Output is multiplyed by 10 by default due to the 
   * integrated encoders default 100ms sampling time. Thus returning
   * meters every second and instead of centimeters every second. 
   * @return velocity m/s
   */
  public double getRightEncoderDistanceRate() {
    return (rightFront.getSelectedSensorVelocity(0) * DrivetrainConstants.DistancePerPulse) * 10;
  }

  /**
   * Gets the left distance traveled according to the ammount
   * of encoder rotations that are counted.
   * @return distance in meters traveled
   */
  public double getLeftEncoderDistance() {
    return this.getLeftEncoderValue() * DrivetrainConstants.DistancePerPulse;
  }

  /**
   * Gets the right distance traveled according to the ammount
   * of encoder rotations that are counted.
   * @return distance in meters traveled
   */
  public double getRightEncoderDistance() {
    return this.getRightEncoderValue() * DrivetrainConstants.DistancePerPulse;
  }

  /**
   * Gets total avg distance traveled according to the
   * average distance traveled by the left and right
   * encoders.
   * @return average distance in meters traveled
   */
  public double getAvgEncoderDistance() {
    return (this.getLeftEncoderDistance() + this.getRightEncoderDistance())/2;
  }

  /**
   * Drives the both sides of the robot drive motors
   * directly with voltage.
   * @param leftVolts
   * @param rightVolts
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftBack.setVoltage(leftVolts);
    rightBack.setVoltage(-rightVolts);
    drive.feed();
  }

  /**
   * Zero the angle (the yaw) of the gyro
   */
  public void zeroAngle() {
    gyro.setYaw(0);
  }

  /**
   * Gets the angle (the yaw) of the gyro
   * @return angle in degrees
   */
  public double getAngle() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return Math.IEEEremainder(ypr[0], 360) * (DrivetrainConstants.GyroReversed ? -1.0 : 1.0);
  }

  /**
   * Gets the velocity of the angle of the gyro in
   * degress per second
   * @return degrees per second
   */
  public double getTurnRate() {
    double[] dps = new double[3];
    gyro.getRawGyro(dps);
    return dps[0] * (DrivetrainConstants.GyroReversed ? -1.0 : 1.0);
  }

  /**
   * Gets the total encoder pulses from the left encoder.
   * @return encoder pulse total
   */
  public double getLeftEncoderValue() {
    return leftFront.getSelectedSensorPosition(0);
  }

  /**
   * Gets the total encoder pulses from the right encoder.
   * @return encoder pulse total
   */
  public double getRightEncoderValue() {
    return rightFront.getSelectedSensorPosition(0);
  }

  /**
   * Resets encoder pulse count.
   */
  public void resetEncoders() {
    this.rightFront.setSelectedSensorPosition(0);
    this.leftFront.setSelectedSensorPosition(0);
  }

  /**
   * Sets wether to use ramp or not.
   * @param status
   */
  public void setRamp(boolean status) {
    if (status == true) {
      this.rightFront.configOpenloopRamp(DrivetrainConstants.secondsFromNeutralToFull, 10);
      this.leftFront.configOpenloopRamp(DrivetrainConstants.secondsFromNeutralToFull, 10);
    } else {
      this.rightFront.configOpenloopRamp(0, 10);
      this.leftFront.configOpenloopRamp(0, 10);
    }
  }

  /**
   * Will set max allowable speed for the drive train.
   * @param output
   */
  public void setMaxOutput(double output) {
    drive.setMaxOutput(output);
  }

  /**
   * Gets the current pose (odometery position) of the robot.
   * @return Pose2d
   */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  /**
   * Gets the current wheel speed using encoderDistanceRate
   * @return DifferentialDriveWheelSpeeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderDistanceRate(), getRightEncoderDistance());
  }

  /**
   * Resets the Odometery position with the specified pose
   * @param pose2d
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
  }
}
