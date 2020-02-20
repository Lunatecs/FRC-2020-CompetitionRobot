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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax leftFlywheel = new CANSparkMax(ShooterConstants.Flywheel_Left_ID, MotorType.kBrushless);
  private final CANSparkMax rightFlywheel = new CANSparkMax(ShooterConstants.Flywheel_Right_ID, MotorType.kBrushless);
  private final DoubleSolenoid hood = new DoubleSolenoid(ShooterConstants.Hood_Forward_ID, ShooterConstants.Hood_Reverse_ID);

  private boolean isLowered = true;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {

    leftFlywheel.restoreFactoryDefaults();
    leftFlywheel.setIdleMode(IdleMode.kCoast);
    leftFlywheel.setInverted(true);

    rightFlywheel.restoreFactoryDefaults();
    rightFlywheel.setIdleMode(IdleMode.kCoast);
    
    this.isLowered = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the flywheel.
   * @param speed
   */
  public void setFlyWheelSpeed(double speed) {
    leftFlywheel.set(speed);
    rightFlywheel.set(speed);
  }

  /**
   * Lowers the hood of the shooter.
   */
  public void lowerHood() {
    hood.set(DoubleSolenoid.Value.kForward);
    this.isLowered = true;
  }

  /**
   * Raises the hood of the shooter.
   */
  public void raiseHood() {
    hood.set(DoubleSolenoid.Value.kReverse);
    this.isLowered = false;
  }

  /**
   * The status of the hood.
   * @return if the hood is lowered or not
   */
  public boolean isLowered() {
    return this.isLowered;
  }
}
