/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import frc.robot.Constants.CharacterizationConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PathFollowingConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.buttons.JoystickAxisButton;
import frc.robot.commands.AdjustFlyWheelCommand;
import frc.robot.commands.CurvatureWithJoysticksCommand;
import frc.robot.commands.ScanForTargetCommand;
import frc.robot.commands.DoNothingAutoCommand;
import frc.robot.commands.DriveWithJoysticksCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.FeederSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Joystick driverJoystick = new Joystick(ControllerConstants.Joystick_USB_Driver);
  private final Joystick operatorJoystick = new Joystick(ControllerConstants.Joystick_USB_Operator);
  private final DrivetrainSubsystem driveTrain = new DrivetrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final TowerSubsystem tower = new TowerSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final ManualTurretCommand manualTurret = new ManualTurretCommand(() -> this.operatorJoystick.getRawAxis(ControllerConstants.Joystick_Left_X_Axis), turret);
  private final ScanForTargetCommand scanForTarget = new ScanForTargetCommand(turret, limelight);

  private String driveSelected;
  private final SendableChooser<String> driveChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //autoChooser.setDefaultOption("Pathfind-1", object);
    // Configure the button bindings
    configureDefaultCommands();
  }

  public void configureAutos() {
    final DoNothingAutoCommand doNothing = new DoNothingAutoCommand();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(PathFollowingConstants.pathfinding1JSON);
      Trajectory trajectoryPathweaver = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      final RamseteCommand pathfollow1 = new RamseteCommand(
        trajectoryPathweaver,
        driveTrain::getPose,
        new RamseteController(PathFollowingConstants.RamseteB, PathFollowingConstants.RamseteZeta),
        new SimpleMotorFeedforward(CharacterizationConstants.ksVolts,
                                   CharacterizationConstants.kvVoltSecondsPerMeter,
                                   CharacterizationConstants.kaVoltsSecondsSquaredPerMeter),
        CharacterizationConstants.DriveKinematics,
        driveTrain::getWheelSpeeds,
        new PIDController(CharacterizationConstants.kPDriveVel, 0, 0),
        new PIDController(CharacterizationConstants.kPDriveVel, 0, 0),
        driveTrain::tankDriveVolts,
        driveTrain);
      autoChooser.addOption("Pathfollow1", pathfollow1);
    } catch (IOException e) {
      DriverStation.reportError("Unable to access trajectory: " + PathFollowingConstants.pathfinding1JSON, e.getStackTrace());
    }
    
    autoChooser.setDefaultOption("Do Nothing", doNothing);
    SmartDashboard.putData("Auto choises", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureOperatorButtonBindings() {

    new JoystickButton(operatorJoystick, ControllerConstants.Green_Button_ID).whenPressed(() -> driveTrain.zeroAngle());

    // new JoystickButton(operatorJoystick, ControllerConstants.Right_Bumper_ID)
    //                                                                  .whenPressed(() -> shooter.setFlyWheelSpeed(.9))
    //                                                                  .whenReleased(() -> shooter.setFlyWheelSpeed(0));


    new JoystickButton(operatorJoystick, ControllerConstants.Left_Bumper_ID)
                                                                    .whenPressed(() -> {feeder.setFeederSpeed(-1);
                                                                                        tower.setConveyorSpeed(1);})
                                                                    .whenReleased(() -> {feeder.setFeederSpeed(0);
                                                                                        tower.setConveyorSpeed(0);});
    
    new POVButton(operatorJoystick, 0).whileHeld(new AdjustFlyWheelCommand(shooter, 4500));
//                                      .whenReleased(new InstantCommand(() -> shooter.setFlyWheelSpeed(0), shooter));

    new POVButton(operatorJoystick, 180).whileHeld(new AdjustFlyWheelCommand(shooter, 5500));
//                                      .whenReleased(new InstantCommand(() -> shooter.setFlyWheelSpeed(0), shooter));

    new JoystickButton(operatorJoystick, ControllerConstants.Blue_Button_ID).whenPressed(() ->
    { 
      if (shooter.isLowered()) {
        shooter.raiseHood();
      } else {
        shooter.lowerHood();
      }
    });

    new JoystickButton(operatorJoystick, ControllerConstants.Red_Button_ID).whileActiveContinuous(scanForTarget); 

  }

  private void configureDefaultCommands() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.setDefaultCommand(turret, manualTurret);
    scheduler.setDefaultCommand(climber, new RunCommand(() -> 
    { 
      if(!climber.isHighStageRetracted()) {
        climber.setHighStage(MathUtil.clamp(this.operatorJoystick.getRawAxis(ControllerConstants.Joystick_Right_Y_Axis),-ClimberConstants.HighStageMaxSpeed,ClimberConstants.HighStageMaxSpeed));  
      } else {
        climber.setHighStage(0.0);
      }
    },
    this.climber));
  }

  public void configureDriverButtonBindings(String drive) {
    new JoystickButton(driverJoystick, ControllerConstants.Right_Bumper_ID)
                                                                          .whenPressed(() -> driveTrain.setMaxOutput(0.25))
                                                                          .whenReleased(() -> driveTrain.setMaxOutput(.85));

    new JoystickButton(driverJoystick, ControllerConstants.Left_Bumper_ID)
                                                                          .whenPressed(() -> driveTrain.setMaxOutput(.5))
                                                                          .whenReleased(() -> driveTrain.setMaxOutput(.85));

    new JoystickButton(driverJoystick, ControllerConstants.Yellow_Button_ID).whenPressed(() -> 
    { 
      if (intake.isLowered()) {
        intake.raiseIntake();
      } else {
        intake.lowerIntake();
      }
    });

    new JoystickButton(driverJoystick, ControllerConstants.Green_Button_ID).whenPressed(() -> 
    {
      if (climber.isLowered()) {
        climber.raiseLowStage();
      } else {
        climber.lowerLowStage();
      }
    });

    if(drive=="Arcade") {
      new JoystickAxisButton(driverJoystick, ControllerConstants.Right_Trigger_ID)
                                                                      .whenPressed(() -> intake.setIntakeSpeed(1))
                                                                      .whenReleased(() -> intake.setIntakeSpeed(0));

      new JoystickAxisButton(driverJoystick, ControllerConstants.Left_Trigger_ID)
                                                                      .whenPressed(() -> intake.setIntakeSpeed(-1))
                                                                      .whenReleased(() -> intake.setIntakeSpeed(0));
    } else {
      new JoystickButton(driverJoystick, ControllerConstants.Green_Button_ID).whenPressed(() -> intake.setIntakeSpeed(1))
                                                                            .whenReleased(() -> intake.setIntakeSpeed(0));
      //new JoystickButton(driverJoystick, ControllerConstants.Blue_Button_ID).whenPressed(() -> intake.lowerIntake());
      new JoystickButton(driverJoystick, ControllerConstants.Red_Button_ID).whenPressed(()-> intake.setIntakeSpeed(-1))
                                                                            .whenReleased(() -> intake.setIntakeSpeed(0));
    }
  }
  
  public void configureDriveDefault() {
    final DriveWithJoysticksCommand joystickDrive = new DriveWithJoysticksCommand(driveTrain, 
                                                                        () -> { return -driverJoystick.getRawAxis(ControllerConstants.Joystick_Left_Y_Axis);}, 
                                                                     () -> { return driverJoystick.getRawAxis(ControllerConstants.Joystick_Right_X_Axis);});
    final CurvatureWithJoysticksCommand curvatureDrive = new CurvatureWithJoysticksCommand(driveTrain,
                                                                        () -> { return -driverJoystick.getRawAxis(ControllerConstants.Left_Trigger_ID) + driverJoystick.getRawAxis(ControllerConstants.Right_Trigger_ID);},
                                                                        () -> 
                                                                        { 
                                                                          if ((Math.abs(driverJoystick.getRawAxis(ControllerConstants.Joystick_Left_X_Axis)) >= Math.abs((driverJoystick.getRawAxis(ControllerConstants.Joystick_Right_X_Axis))))) {
                                                                            return driverJoystick.getRawAxis(ControllerConstants.Joystick_Left_X_Axis);
                                                                          } else {
                                                                            return driverJoystick.getRawAxis(ControllerConstants.Joystick_Right_X_Axis);
                                                                          }
                                                                        }, 
                                                                        () -> { return driverJoystick.getRawButton(ControllerConstants.Blue_Button_ID);});

    
    driveChooser.setDefaultOption("Arcade", "Arcade");
    driveChooser.addOption("Curve", "Curve");
    SmartDashboard.putData("Drive choices", driveChooser);

    CommandScheduler scheduler = CommandScheduler.getInstance();
    driveSelected = driveChooser.getSelected();
    
    if (driveSelected == "Arcade") {
      configureDriverButtonBindings(driveSelected);
      configureOperatorButtonBindings();
      scheduler.setDefaultCommand(driveTrain, joystickDrive);
    } else if(driveSelected == "Curve") {
      configureDriverButtonBindings(driveSelected);
      configureOperatorButtonBindings();
      scheduler.setDefaultCommand(driveTrain, curvatureDrive);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
