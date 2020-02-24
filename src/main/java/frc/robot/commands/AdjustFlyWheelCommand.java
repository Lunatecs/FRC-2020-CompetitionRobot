/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AdjustFlyWheelCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final double setpoint;
  PIDController velocityControl = new PIDController(0.0009, 0.0, 0.00009);
  private boolean upToSpeed = false;
  private boolean isFinish = false;
  /**
   * Creates a new AdjustFlyWheel.
   */
  public AdjustFlyWheelCommand(ShooterSubsystem shooter, double setpoint) {
    this.shooter = shooter;
    this.setpoint = setpoint;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    velocityControl.setSetpoint(this.setpoint);
    velocityControl.setTolerance(10);
    isFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   /* if( upToSpeed && shooter.getAvgVelocity() < setpoint/2.0) {
      upToSpeed = false;
      shooter.setFlyWheelSpeed(.9);
    } else if(shooter.getAvgVelocity()>=setpoint || upToSpeed) {
      upToSpeed = true;
      shooter.setFlyWheelSpeed(velocityControl.calculate(shooter.getAvgVelocity()));
    } else {
      shooter.setFlyWheelSpeed(.9);
    }
    SmartDashboard.putBoolean("up to speed", upToSpeed);
  */

  double speed = velocityControl.calculate(shooter.getAvgVelocity(),setpoint);

  shooter.setFlyWheelSpeed(speed);

  SmartDashboard.putNumber("ShooterSpeed", speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinish = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinish) {
      shooter.setFlyWheelSpeed(0);
    }
    return isFinish;
  }

  @Override
  public void cancel() {
    isFinish = true;
  }
}
