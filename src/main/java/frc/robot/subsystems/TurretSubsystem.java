/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private final WPI_TalonSRX turret = new WPI_TalonSRX(TurretConstants.Turret_ID);
  private NeutralMode TURRET_NEUTRALMODE = NeutralMode.Brake;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    turret.configFactoryDefault();
    turret.setNeutralMode(TURRET_NEUTRALMODE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the turret.
   * @param speed
   */
  public void setTurretSpeed(double speed) {
    turret.set(ControlMode.PercentOutput, speed);
  }
}
