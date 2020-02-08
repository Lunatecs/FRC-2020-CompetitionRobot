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

import static frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final DoubleSolenoid intakeWrist = new DoubleSolenoid(IntakeConstants.Intake_Forward_ID, 
                                                                IntakeConstants.Intake_Backward_ID);
  
  private final WPI_VictorSPX rollers = new WPI_VictorSPX(IntakeConstants.Rollers_ID);
  private static NeutralMode INTAKE_NEUTRALMODE = NeutralMode.Brake;
  
  private boolean isLowered = false;
  
  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    rollers.configFactoryDefault();
    rollers.setNeutralMode(INTAKE_NEUTRALMODE);
    this.isLowered = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the roller speed on the intake.
   * @param speed
   */
  public void setIntakeSpeed(double speed) {
    rollers.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Raises intake using the piston.
   */
  public void raiseIntake() {
    intakeWrist.set(DoubleSolenoid.Value.kReverse);
    this.isLowered = false;
  }
  /**
   * Lowers intake using the piston.
   */
  public void lowerIntake() {
    intakeWrist.set(DoubleSolenoid.Value.kForward);
    this.isLowered = true;
  }

  /**
   * Gets status of the intake
   * @return if intake is lowered or not
   */
  public boolean isLowered() {
    return this.isLowered;
  }
}
