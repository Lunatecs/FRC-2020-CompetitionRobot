/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.TowerConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TowerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX conveyor = new WPI_TalonSRX(TowerConstants.Conveyor_ID);

  private static NeutralMode TOWER_NEUTRALMODE = NeutralMode.Brake;
  
  /**
   * Creates a new Tower.
   */
  public TowerSubsystem() {
    conveyor.configFactoryDefault();
    conveyor.setNeutralMode(TOWER_NEUTRALMODE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the conveyor speed of the tower
   * @param speed
   */
  public void setConveyorSpeed(double speed) {
    conveyor.set(ControlMode.PercentOutput, speed);
  }
}
