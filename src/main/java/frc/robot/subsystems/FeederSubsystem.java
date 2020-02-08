/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.FeederConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

  private final WPI_VictorSPX guiders = new WPI_VictorSPX(FeederConstants.Guiders_ID);
  private static NeutralMode FEEDER_NEUTRALMODE = NeutralMode.Brake;

  /**
   * Creates a new FeederSubsystem.
   */
  public FeederSubsystem() {
    guiders.configFactoryDefault();
    guiders.setNeutralMode(FEEDER_NEUTRALMODE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * Sets the guiders speed on the feeder.
   * @param speed
   */
  public void setFeederSpeed(double speed) {
    guiders.set(ControlMode.PercentOutput, speed);
  }
}
