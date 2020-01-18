package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickDrive extends Command {
    public static final Joystick JOYSTICK = OI.getDriverJoystick();
    //public static final Trigger TRIGGER = OI.getDriverTrigger();
    public static final int THROTTLE = OI.AXIS_LEFT_STICK_Y;
    public static final int ROTATE = OI.AXIS_RIGHT_STICK_X;
    //public static GoForward goForward = new GoForward();
    //public static Turn turn = new Turn(1);
    double target_yaw = 0.0;
    //public static final int TRIGGER = OI.AXIS_LEFT_TRIGGER;

    public JoystickDrive() {
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.driveTrain.stop();
        
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        /*if (JOYSTICK.getRawButtonPressed(2)) {
            goForward.start();
            
        }
        if (JOYSTICK.getRawButtonPressed(5)) {
            turn.start();
            
        }
        if (JOYSTICK.getRawButton(3)) {
        if (Robot.Paused)	{
        Robot.Paused = false;
        } else {
            Robot.Paused = true;

        }
            
        
        }*/

        Robot.driveTrain.throttle = Robot.driveTrain.globalThrottle;
        Robot.driveTrain.rotate = Robot.driveTrain.globalRotate;
        //Robot.driveTrain.throttle = -JOYSTICK.getRawAxis(THROTTLE);
        //Robot.driveTrain.rotate = JOYSTICK.getRawAxis(ROTATE);

        double offset = 0;
        double speed = 4; // the speed limit

          //Allows the driver to change the driving mode of the bot 
        Robot.driveTrain.DRIVE_MODE = 0;
        
        if(JOYSTICK.getRawButton(OI.BUTTON_B)) {
            if(Robot.driveTrain.DRIVE_MODE == 0) {
                Robot.driveTrain.DRIVE_MODE = 1; //0 = setpower(i.e Voltage Mode), 1 = ratedrive(i.e Close Loop on Talon)
            } else {
                Robot.driveTrain.DRIVE_MODE = 0; //0 = setpower(i.e Voltage Mode), 1 = ratedrive(i.e Close Loop on Talon)
            }
        } 
        

        if(Robot.driveTrain.throttle < 0.25 && Robot.driveTrain.throttle > -0.25) {
            Robot.driveTrain.throttle = 0;
        }
        if(Robot.driveTrain.rotate < 0.25 && Robot.driveTrain.rotate > -0.25) {
            Robot.driveTrain.rotate = 0;
        }
        
        if(Robot.driveTrain.DRIVE_MODE == 0) {
            /*boolean tank = false;   
            if (throttle < .25) {
                tank = true;
            }*/
            //Robot.driveTrain.pidTuneTeleop(THROTTLE, ROTATE, target_yaw);
            //Robot.driveTrain.setPower(Robot.driveTrain.throttle * ((-JOYSTICK.getZ() + 1) / 2), Robot.driveTrain.rotate * ((-JOYSTICK.getZ() + 1) / 2), .85, JOYSTICK.getRawButton(1));
            Robot.driveTrain.setPower(-JOYSTICK.getRawAxis(THROTTLE), JOYSTICK.getRawAxis(ROTATE), 0.5, (JOYSTICK.getRawAxis(3) > 0.4)); //sens = 0.75

            //limelight Code for Arcade Drive
            if (JOYSTICK.getRawButton(OI.BUTTON_A) == true)	{
                target_yaw = -20 * Limelight.getLimelightX();
            }
            else{
                target_yaw = 0.0;
            }

        } else if(Robot.driveTrain.DRIVE_MODE == 1) {
            // Robot.driveTrain.rateDrive(Robot.driveTrain.throttle, Robot.driveTrain.rotate, offset, speed);
            if (JOYSTICK.getRawButton(OI.BUTTON_X)) {
                offset = Robot.driveTrain.limelightPID.execute(0, Limelight.getLimelightX());
            }
            else {
                offset = 0.0;
            }
            offset = 0.0;
            Robot.driveTrain.rateDriveImproved(Robot.driveTrain.throttle, Robot.driveTrain.rotate, offset, speed);
        }
    }

        // Make this return true when this Command no longer needs to run execute()
        protected boolean isFinished() {
            return false;
        }

        // Called once after isFinished returns true
        protected void end() {
            Robot.driveTrain.stop();
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void interrupted() {
            end();
        }
}
