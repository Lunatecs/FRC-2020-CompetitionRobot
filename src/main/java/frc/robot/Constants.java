/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DrivetrainConstants {
        public static final int Right_Front_ID = 12;
        public static final int Right_Back_ID = 13;

        public static final int Left_Front_ID = 14;
        public static final int Left_Back_ID = 15;

        public static final double DistancePerPulse = 0.0000202129;
        public static final boolean GyroReversed = false;

        public static final double secondsFromNeutralToFull = 0.25;
    }

    public static final class CharacterizationConstants {
        public static final double TrackWidthMeters = 0.636;
        public static final DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(TrackWidthMeters);

        public static final double MaxSpeedMetersPerSecond = 3.6;
        public static final double MaxAccelerationMetersPerSecondSquared = 3.1;

        public static final double ksVolts = 0.48;
        public static final double kvVoltSecondsPerMeter = 2.57;
        public static final double kaVoltsSecondsSquaredPerMeter = 0.271;

        public static final double kPDriveVel = 10.6;

    }

    public static final class PathFollowingConstants {

        public static final double RamseteB = 2.0;
        public static final double RamseteZeta = 0.7;

        public static final String pathfinding1JSON = "paths/test.wpilib.json";
    }

    public static final class ColorWheelConstants {
        public static final int ColorWheel_ID = 3;
    }

    public static final class IntakeConstants {
        public static final int Rollers_ID = 6;
        public static final int Intake_Forward_ID = 0;
        public static final int Intake_Backward_ID = 7;
    }

    public static final class FeederConstants {
        public static final int Guiders_Left_ID = 5;
        public static final int Guiders_Right_ID = 9;
    }

    public static final class TowerConstants {
        public static final int Conveyor_ID = 2;
    }

    public static final class TurretConstants {
        public static final int Turret_ID = 16;

        public static final double MinSpeed = 0.05;
        public static final double ScanSpeed = 0.25;
        
        public static final int FwdMaxSensorPostion = 11000;
        public static final double FwdKp = 0.00015;
        public static final double FwdKd = 0.0;
        public static final double FwdKi = 0.0;
        
        public static final int BckMaxSensorPostion = -12000;
        public static final double BckKp = 0.00015;
        public static final double BckKd = 0.0;
        public static final double BckKi = 0.0;
    }

    public static final class ShooterConstants {
        public static final int Flywheel_Left_ID = 1;
        public static final int Flywheel_Right_ID = 2;
        public static final int Hood_Forward_ID = 1;
        public static final int Hood_Reverse_ID = 2;
    }

    public static final class ControllerConstants {
        public static final int Joystick_USB_Driver = 0;
        public static final int Joystick_USB_Operator = 1;

        public static final int Joystick_Right_X_Axis = 4;
        public static final int Joystick_Right_Y_Axis = 5;
        public static final int Joystick_Left_X_Axis = 0;
        public static final int Joystick_Left_Y_Axis = 1;

        public static final int Joystick_Right_Button_ID = 10;
        public static final int Joystick_Left_Button_ID = 9;

        public static final int Red_Button_ID = 2;
        public static final int Green_Button_ID = 1;
        public static final int Yellow_Button_ID = 4;
        public static final int Blue_Button_ID = 3;

        public static final int Left_Bumper_ID = 5;
        public static final int Right_Bumper_ID = 6;

        public static final int Left_Trigger_ID = 2;
        public static final int Right_Trigger_ID = 3;

        public static final int Left_Select_ID = 7;
        public static final int Right_Select_ID = 8;
    }
}
