/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FlyWheelCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final double setpoint;
  private boolean isFinish = false;
  /**
   * Creates a new FlyWheelCommand.
   */
  public FlyWheelCommand(ShooterSubsystem shooter, double setpoint) {
    this.shooter = shooter;
    this.setpoint = setpoint;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSetpoint(this.setpoint);
    shooter.enable();
    isFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
      shooter.disable();
    }
    return isFinish;
  }

  @Override
  public void cancel() {
    isFinish = true;
  }
}
