package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {

    public static double distanceFromPort = 0;   // x distance from the port
    public static double distanceShot = 0;
    public static double pythagoreanShotDistance = 0; // 

    public static double height1 = 0;   // y height from the floor to the camera in meters
    public static double height2 = 2.5019;   // y height from the floor to the port height in meters
    public static double angle1 = 0;   // mounting angle of the limelight
    public static double angle2 = 0;   // y angle to the target that limelight tells you
  
    public static double angleOfShooter = 0;   // angle of the hood of the shooter 
    public static double shooterPower = 0;    // the power value that is sent to the flywheel shooter motor

  /*************************************************************************
  * This method gives the x distance from the shooter to the outer port.
  * 
  *************************************************************************/
public static double getDistanceToPort() {
    distanceFromPort = (height2-height1) / Math.tan(angle1 + angle2);
    return distanceFromPort;
}
  /*************************************************************************
  * This method gives the hypoteneuse distance from the shooter 
  * to the outer port.
  *************************************************************************/
public static double getDistanceShot() {
    pythagoreanShotDistance = (Math.pow(height1, 2) + Math.pow(distanceFromPort, 2));
    distanceShot = Math.sqrt(pythagoreanShotDistance);
    return distanceShot;
}
  /*************************************************************************
  * Add your docs here.
  * 
  *************************************************************************/
public static double getAngleOfShooter() {
    Math.atan((height1/distanceFromPort));
    return angleOfShooter;
}
  /*************************************************************************
  * Add your docs here.
  * 
  *************************************************************************/
@Override
protected void initDefaultCommand() {

}

}