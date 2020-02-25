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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

public class ShooterSubsystem extends PIDSubsystem {

  private final CANSparkMax leftFlywheel = new CANSparkMax(ShooterConstants.Flywheel_Left_ID, MotorType.kBrushless);
  private final CANSparkMax rightFlywheel = new CANSparkMax(ShooterConstants.Flywheel_Right_ID, MotorType.kBrushless);
  private final DoubleSolenoid hood = new DoubleSolenoid(ShooterConstants.Hood_Forward_ID, ShooterConstants.Hood_Reverse_ID);

  private boolean isLowered = true;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    //0.0009,0 0.00009
    super(new PIDController(0.0009, 0.0, 0.00009), 0);

    leftFlywheel.restoreFactoryDefaults();
    leftFlywheel.setIdleMode(IdleMode.kCoast);
    leftFlywheel.setInverted(true);

    rightFlywheel.restoreFactoryDefaults();
    rightFlywheel.setIdleMode(IdleMode.kCoast);
    this.getController().setTolerance(50);
    this.isLowered = true;
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Left Velocity", this.getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Velocity", this.getRightEncoderVelocity());

    SmartDashboard.putNumber("AVG Velocity", (this.getRightEncoderVelocity() + this.getRightEncoderVelocity())/2.0);
    // This method will be called once per scheduler run
  }

  /**
   * Gets the status of the PID controller stating wether
   * we are at our specific setpoint.
   * @return PID controller at set point w/ tolerence
   */
  public boolean atSetpoint() {
    return this.getController().atSetpoint();
  }

  @Override
  public double getMeasurement() {
    return this.getAvgVelocity();
  }

  /**
   * Uses the pid loop output and applys it to the motor
   */
  @Override
  public void useOutput(double output, double setPoint) {
    this.setFlyWheelSpeed(output);
    SmartDashboard.putString("PID Output", output + ":" + this.getController().getSetpoint());
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

  public double getLeftEncoderVelocity() {
    return leftFlywheel.getEncoder().getVelocity();
  }


  public double getRightEncoderVelocity() {
    return rightFlywheel.getEncoder().getVelocity();
  }

  public double getAvgVelocity() {
    return (this.getRightEncoderVelocity() + this.getRightEncoderVelocity())/2.0;
  }

  /**
   * The status of the hood.
   * @return if the hood is lowered or not
   */
  public boolean isLowered() {
    return this.isLowered;
  }
}
