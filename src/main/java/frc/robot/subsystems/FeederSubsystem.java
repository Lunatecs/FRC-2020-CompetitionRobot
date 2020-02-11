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

  private final WPI_VictorSPX left_guiders = new WPI_VictorSPX(FeederConstants.Guiders_Left_ID);
  private final WPI_VictorSPX right_guiders = new WPI_VictorSPX(FeederConstants.Guiders_Right_ID);
  private static NeutralMode FEEDER_NEUTRALMODE = NeutralMode.Brake;

  /**
   * Creates a new FeederSubsystem.
   */
  public FeederSubsystem() {
    left_guiders.configFactoryDefault();
    right_guiders.configFactoryDefault();
    left_guiders.setNeutralMode(FEEDER_NEUTRALMODE);
    right_guiders.setNeutralMode(FEEDER_NEUTRALMODE);
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
    left_guiders.set(ControlMode.PercentOutput, speed);
    right_guiders.set(ControlMode.PercentOutput, speed);
  }
}
