/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveWithJoysticksCommand extends CommandBase {
  private final DrivetrainSubsystem driveTrain;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;

  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoysticksCommand(DrivetrainSubsystem driveTrain, DoubleSupplier speed, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.speed = speed;
    this.rotation = rotation;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Makes deadzone
    if (Math.abs(speed.getAsDouble()) > 0.15 || Math.abs(rotation.getAsDouble()) > 0.15) {
      driveTrain.arcadeDrive((speed.getAsDouble()), rotation.getAsDouble());
    }else{
      driveTrain.arcadeDrive(0,0);
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
