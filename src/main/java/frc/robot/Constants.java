/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

        public static final double secondsFromNeutralToFull = 0.25;
    }

    public static final class IntakeConstants {
        public static final int Rollers_ID = 0;
        public static final int Intake_Forward_ID = 0;
        public static final int Intake_Backward_ID = 0;
    }

    public static final class FeederConstants {
        public static final int Guiders_ID = 0;
    }

    public static final class TowerConstants {
        public static final int Conveyor_ID = 0;
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
