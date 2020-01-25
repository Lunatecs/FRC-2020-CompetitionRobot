/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.DrivetrainConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftFront = new WPI_TalonFX(DrivetrainConstants.Left_Front_ID);
  private final WPI_TalonFX leftBack = new WPI_TalonFX(DrivetrainConstants.Left_Back_ID);

  private final WPI_TalonFX rightFront = new WPI_TalonFX(DrivetrainConstants.Right_Front_ID);
  private final WPI_TalonFX rightBack = new WPI_TalonFX(DrivetrainConstants.Right_Back_ID);

  private static NeutralMode DRIVE_NEUTRALMODE = NeutralMode.Brake;

  private DifferentialDrive drive;
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

    drive = new DifferentialDrive(leftFront, rightFront);
    
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public double getLeftEncoder() {
    return leftFront.getSelectedSensorPosition(0);
  }

  public double getRightEncoder() {
    return rightFront.getSelectedSensorPosition(0);
  }

  public void resetEncoders() {
    this.rightFront.setSelectedSensorPosition(0);
    this.leftFront.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("R Encoder", this.getRightEncoder());
    SmartDashboard.putNumber("L Encoder", this.getLeftEncoder());
    // This method will be called once per scheduler run
  }
}
