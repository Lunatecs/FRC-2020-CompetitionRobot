/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CharacterizationConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAutoCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new ShootAutoCommandGroup.
   */
  public ShootAutoCommandGroup(double meters, 
                              PIDController turretPIDController,
                              SimpleMotorFeedforward feedforward,
                              FeederSubsystem feeder, 
                              LimelightSubsystem limelight, 
                              ShooterSubsystem shooter, 
                              DrivetrainSubsystem driveTrain, 
                              TowerSubsystem tower,
                              TurretSubsystem turret,
                              IntakeSubsystem intake,
                              boolean threemore) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new InstantCommand(() intake.lowerIntake()),
      new InstantCommand(() -> {shooter.setSetpoint(3650); shooter.enable();}),
      new PIDCommand(
        turretPIDController, limelight::getTX, 0, output -> turret.setTurretSpeed(output, true), turret),
      new WaitUntilCommand(shooter::atSetpoint),
      new WaitUntilCommand(limelight::isOnTarget),
      new InstantCommand(() -> turret.lock()),
      new InstantCommand(() -> {feeder.setFeederSpeed(-1); tower.setConveyorSpeed(1); SmartDashboard.putBoolean("IsHere", true);}),
      new WaitCommand(2),
      new InstantCommand(() -> {shooter.disable(); tower.setConveyorSpeed(0); turret.unLock(); feeder.setFeederSpeed(0);}));

    if(threemore) {
      this.addCommands(
        new InstantCommand(() -> {shooter.setSetpoint(4650); shooter.enable();}),
        new InstantCommand(() -> intake.setIntakeSpeed(-1)));
    }

    this.addCommands(
      new TrapezoidProfileCommand(
              new TrapezoidProfile(
                  // Limit the max acceleration and velocity
                  new TrapezoidProfile.Constraints(
                   CharacterizationConstants.MaxSpeedMetersPerSecond,
                   CharacterizationConstants.MaxAccelerationMetersPerSecondSquared),
                  // End at desired position in meters; implicitly starts at 0
                  new TrapezoidProfile.State(meters, 0)),
              // Pipe the profile state to the drive
              setpointState -> driveTrain.arcadeDrive(-feedforward.calculate(setpointState.velocity)/12.0, 0),
              // Require the drive
              driveTrain).beforeStarting(() -> driveTrain.resetEncoders(), driveTrain)
    );

    if(threemore) {
      this.addCommands(
        new WaitUntilCommand(shooter::atSetpoint),
        new WaitUntilCommand(limelight::isOnTarget),
        new InstantCommand(() -> {feeder.setFeederSpeed(-1); tower.setConveyorSpeed(1); SmartDashboard.putBoolean("IsHere", true);}),
        new WaitCommand(5),
        new InstantCommand(() -> {shooter.disable(); tower.setConveyorSpeed(0); turret.unLock(); feeder.setFeederSpeed(0);})
      );
    }


  }
}
