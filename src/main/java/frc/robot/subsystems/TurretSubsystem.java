/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private final TalonSRX turret = new TalonSRX(TurretConstants.Turret_ID);
  private NeutralMode TURRET_NEUTRALMODE = NeutralMode.Brake;
  private PIDController pidController;
  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    turret.configFactoryDefault();
    turret.setNeutralMode(TURRET_NEUTRALMODE);
    resetPostion();
    pidController = new PIDController(.001,0,0);
    pidController.setSetpoint(10000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Encoder", getPostion());
  }

  /**
   * Sets the speed of the turret.
   * @param speed
   */
  public void setTurretSpeed(double speed) {
    pidController.calculate(this.getPostion());
    turret.set(ControlMode.PercentOutput, speed);
  }

  public int getPostion() {
    return this.turret.getSelectedSensorPosition(0);
  }

  public void resetPostion() {
    turret.setSelectedSensorPosition(0, 0, 10);
  }
}
