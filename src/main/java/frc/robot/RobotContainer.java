/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ControllerConstants;
import frc.robot.buttons.JoystickAxisButton;
import frc.robot.commands.DriveWithJoysticksCommand;
import frc.robot.commands.ShootWithTriggerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TowerSubsystem;
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
  
  //Negative on Y Axis to invert forward and backward 
  private final DriveWithJoysticksCommand joystickDrive = new DriveWithJoysticksCommand(driveTrain, 
                                                                        () -> { return -driverJoystick.getRawAxis(ControllerConstants.Left_Trigger_ID) + driverJoystick.getRawAxis(ControllerConstants.Right_Trigger_ID);}, 
                                                                        () -> { return driverJoystick.getRawAxis(ControllerConstants.Joystick_Left_X_Axis);});

  private final ShootWithTriggerCommand shootTrigger = new ShootWithTriggerCommand(shooter, () -> { return operatorJoystick.getRawAxis(ControllerConstants.Right_Trigger_ID);});

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(operatorJoystick, ControllerConstants.Right_Bumper_ID)
    //                                                                 .whenPressed(() -> shooter.setFlyWheelSpeed(1))
    //                                                                 .whenReleased(() -> shooter.setFlyWheelSpeed(0));

    new JoystickButton(operatorJoystick, ControllerConstants.Left_Bumper_ID)
                                                                    .whenPressed(() -> {feeder.setFeederSpeed(-1);
                                                                                        tower.setConveyorSpeed(1);})
                                                                    .whenReleased(() -> {feeder.setFeederSpeed(0);
                                                                                        tower.setConveyorSpeed(0);});

    // new JoystickAxisButton(driverJoystick, ControllerConstants.Right_Trigger_ID)
    //                                                                 .whenPressed(() -> driveTrain.setMaxOutput(0.5))
    //                                                                 .whenReleased(() -> driveTrain.setMaxOutput(1));

    // new JoystickAxisButton(driverJoystick, ControllerConstants.Left_Trigger_ID)
    //                                                                 .whenPressed(() -> driveTrain.setMaxOutput(0.25))
    //                                                                 .whenReleased(() -> driveTrain.setMaxOutput(1));


    new JoystickButton(driverJoystick, ControllerConstants.Right_Bumper_ID)
                                                                    .whenPressed(() -> driveTrain.setMaxOutput(0.25))
                                                                    .whenReleased(() -> driveTrain.setMaxOutput(1));

    new JoystickButton(driverJoystick, ControllerConstants.Left_Bumper_ID)
                                                                    .whenPressed(() -> driveTrain.setMaxOutput(.5))
                                                                    .whenReleased(() -> driveTrain.setMaxOutput(1));

    new JoystickButton(driverJoystick, ControllerConstants.Yellow_Button_ID).whenPressed(() -> 
    { 
      if (intake.isLowered()) {
        intake.raiseIntake();
      } else {
        intake.lowerIntake();
      }
    });
    new JoystickButton(driverJoystick, ControllerConstants.Green_Button_ID).whenPressed(() -> intake.setIntakeSpeed(1))
                                                                          .whenReleased(() -> intake.setIntakeSpeed(0));
    //new JoystickButton(driverJoystick, ControllerConstants.Blue_Button_ID).whenPressed(() -> intake.lowerIntake());
    new JoystickButton(driverJoystick, ControllerConstants.Red_Button_ID).whenPressed(()-> intake.setIntakeSpeed(-1))
                                                                          .whenReleased(() -> intake.setIntakeSpeed(0));
  }

  private void configureDefaultCommands() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.setDefaultCommand(driveTrain, joystickDrive);
    scheduler.setDefaultCommand(shooter, shootTrigger);
    scheduler.registerSubsystem(driveTrain);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
