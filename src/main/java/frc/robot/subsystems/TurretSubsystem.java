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
  private PIDController pidControllerFwd;
  private PIDController pidControllerBck;
  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    turret.configFactoryDefault();
    turret.setNeutralMode(TURRET_NEUTRALMODE);
    resetPosition();
    pidControllerFwd = new PIDController(TurretConstants.FwdKp,TurretConstants.FwdKi,TurretConstants.FwdKd);
    pidControllerFwd.setSetpoint(TurretConstants.FwdMaxSensorPostion);
    pidControllerBck = new PIDController(TurretConstants.BckKp,TurretConstants.BckKi,TurretConstants.BckKd);
    pidControllerBck.setSetpoint(TurretConstants.BckMaxSensorPostion);
    turret.setSensorPhase(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Encoder", getPosition());
  }

  /**
   * Sets the speed of the turret. Uses two built in PID Controllers
   * to limit max speed when approaching edges of the turret. One for
   * the left and right o the turret. This helps protect the wiring of
   * the turret.
   * @param speed
   */
  public void setTurretSpeed(double speed) {
    int position = getPosition(); 
    double speedLimitFwd = pidControllerFwd.calculate(position);
    double speedLimitBck = pidControllerBck.calculate(position);
    SmartDashboard.putNumber("speedLimitFwd", speedLimitFwd);
    SmartDashboard.putNumber("speedLimitBck", speedLimitBck);
    
    if(speed > speedLimitFwd && speed > 0) {
      speed = speedLimitFwd;
    } else if(speed < speedLimitBck && speed <= 0) {
      speed = speedLimitBck;
    }
    SmartDashboard.putNumber("actualSpeed", speed);
    turret.set(ControlMode.PercentOutput, speed);
  }

  public int getPosition() {
    return this.turret.getSelectedSensorPosition(0);
  }

  public void resetPosition() {
    turret.setSelectedSensorPosition(0, 0, 10);
  }

  public boolean isFwdLimit() {
    if(getPosition() >= TurretConstants.FwdMaxSensorPostion) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRevLimit() {
    if(getPosition() <= TurretConstants.BckMaxSensorPostion) {
      return true;
    } else {
      return false;
    }
  }
}
