/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
/**
 * Add your docs here.
 */
public class JoystickAxisButton extends Button {
   
    Joystick joystick;
    int axis;

    public JoystickAxisButton(Joystick joystick, int axis) {
        this.joystick = joystick;
        this.axis = axis;
    }
    
    @Override
    public boolean get() {
        return joystick.getRawAxis(this.axis) > 0.2;
    }
}
