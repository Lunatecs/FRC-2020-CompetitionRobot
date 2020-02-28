/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TrackingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScanForTargetCommand extends CommandBase {
  /**
   * Creates a new DefaultTurretCommand.
   */
  double scanSpeed = 0.75;

  TurretSubsystem turret;
  LimelightSubsystem limelight;

  PIDController pController = new PIDController(TrackingConstants.kP, TrackingConstants.kI, TrackingConstants.kD);

  public ScanForTargetCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    pController.setSetpoint(0);
    addRequirements(turret);
  }

  // Called when the command is initially schedSuled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.0;
    SmartDashboard.putBoolean("IsValidTarget", limelight.isValidTarget());
    if(limelight.isValidTarget()) {
      speed = pController.calculate(limelight.getTX(), 0);
      turret.setTurretSpeed(speed, true);
    } else {
      if(turret.isFwdLimit()) {
        scanSpeed = -.4;
      } else if(turret.isRevLimit()) {
        scanSpeed = .4;
      }
      turret.setTurretSpeed(scanSpeed);
    }
    SmartDashboard.putBoolean("Fwd", turret.isFwdLimit());
    SmartDashboard.putBoolean("Rev", turret.isRevLimit());
    SmartDashboard.putNumber("Tx", limelight.getTX());
    SmartDashboard.putNumber("speed", speed);
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
