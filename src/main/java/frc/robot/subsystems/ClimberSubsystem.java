/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final DoubleSolenoid lowStageWrist = new DoubleSolenoid(ClimberConstants.LowStage_Forward_ID, ClimberConstants.LowStage_Reverse_ID);
  private final TalonSRX highStage = new TalonSRX(ClimberConstants.HighStage_ID);
  private final DigitalInput highStageClosedLimitSwitch = new DigitalInput(ClimberConstants.HighStageClosedLimitSwitch);

  private static NeutralMode HIGHSTAGE_NEUTRALMODE = NeutralMode.Brake;
  
  private boolean isLowered = true;
  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {
    highStage.configFactoryDefault();
    highStage.setNeutralMode(HIGHSTAGE_NEUTRALMODE);
    this.isLowered = true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Retracted", this.isHighStageRetracted());
    // This method will be called once per scheduler run
  }

  /**
   * Gets the state of the high stage limit switch to see if it
   * is pressed or not.
   * @return if high stage of climber is retracted
   */
  public boolean isHighStageRetracted() {
    //Have to swap true and false because sensor is normally closed
    if (!highStageClosedLimitSwitch.get()) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * 
   * @param speed
   */
  public void setHighStage(double speed) {
    SmartDashboard.putNumber("Climb Speed", speed);
    highStage.set(ControlMode.PercentOutput, speed);
  }

  public void raiseLowStage() {
    lowStageWrist.set(DoubleSolenoid.Value.kForward);
    this.isLowered = false;
  }

  public void lowerLowStage() {
    lowStageWrist.set(DoubleSolenoid.Value.kReverse);
    this.isLowered = true;
  }

  public boolean isLowered() {
    return this.isLowered;
  }
}
