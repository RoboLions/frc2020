package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;


public class OI {
	public static final int DRIVER_JOYSTICK = 0;
	public static final int MANIPULATOR_JOYSTICK = 1;
	
	public static final int BUTTON_1 = 1;
	public static final int BUTTON_2 = 2;
	public static final int BUTTON_3 = 3;
	public static final int BUTTON_4 = 4;
	public static final int BUTTON_5 = 5;
	public static final int BUTTON_6 = 6;
	public static final int BUTTON_7 = 7;
	public static final int BUTTON_8 = 8;
	public static final int BUTTON_9 = 9;
	public static final int BUTTON_10 = 10;

	public static final int AXIS_LEFT_STICK_X = 0;
	public static final int AXIS_LEFT_STICK_Y = 1;
	public static final int AXIS_LEFT_TRIGGER = 2;
	public static final int AXIS_RIGHT_TRIGGER = 3;
	public static final int AXIS_RIGHT_STICK_X = 4;
	public static final int AXIS_RIGHT_STICK_Y = 5;

	public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int BUTTON_LEFT_BUMPER = 5;
	public static final int BUTTON_RIGHT_BUMPER = 6;
	public static final int BUTTON_BACK = 7;
	public static final int BUTTON_START = 8;
	public static final int BUTTON_LEFT_STICK = 9;
	public static final int BUTTON_RIGHT_STICK = 10;
    
    public static Joystick driverJoystick = new Joystick(DRIVER_JOYSTICK);
    public static Joystick manipulatorJoystick = new Joystick(MANIPULATOR_JOYSTICK);
    
    /*
    Examples of Joysticks and Buttons:
    public final Button autoForwardButton = new JoystickButton(driverJoystick, BUTTON_START);
    */

	public OI() {
        /* 
        Examples whenPressed and whileHeld:
        autoForwardButton.whenPressed(new AutoForward(4.3, 0));
		wheelsOutButton.whileHeld(new SpinWheels(1));
        quickLeftTurn.whenPressed(new AutoPivotHead(5,.2)); 
        */

	}
	
	public static Joystick getDriverJoystick() {
		return driverJoystick;
	}

	public static Joystick getManipulatorJoystick() {
		return manipulatorJoystick;
	}
}