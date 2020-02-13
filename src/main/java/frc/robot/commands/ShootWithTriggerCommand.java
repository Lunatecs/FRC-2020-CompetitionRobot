/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithTriggerCommand extends CommandBase {

  private final ShooterSubsystem shooter;
  private final DoubleSupplier speed;

  /**
   * Creates a new ShootWithJoysticksCommand.
   */
  public ShootWithTriggerCommand(ShooterSubsystem shooter, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.shooter = shooter;
  this.speed = speed;
  addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(speed.getAsDouble()) > 0.15) {
      shooter.setFlyWheelSpeed(speed.getAsDouble());
    } else {
      shooter.setFlyWheelSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
