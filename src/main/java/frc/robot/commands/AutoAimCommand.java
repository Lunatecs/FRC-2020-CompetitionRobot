/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TrackingConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoAimCommand extends CommandBase {
  
  LimelightSubsystem limelight;     
  TurretSubsystem turret;    
  PIDController pidController = new PIDController(0.03, 
                                                  TrackingConstants.kI, 
                                                  TrackingConstants.kD);                                              
  /**
   *
   * Creates a new AutoAimCommand.
   */
  public AutoAimCommand(TurretSubsystem turret, LimelightSubsystem limelight) {
    this.turret = turret;
    this.limelight = limelight;
    this.pidController.setSetpoint(0);
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if(limelight.isValidTarget()) {
      turret.setTurretSpeed(this.pidController.calculate(limelight.getTX()), true);
    } else {
      turret.setTurretSpeed(.2, true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("New End Auto Aim", limelight.isOnTarget());
    return limelight.isOnTarget();
  }
}
