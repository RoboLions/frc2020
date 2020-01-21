/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;


public class AutoLimelightShooter extends Command {
  /* 
  1. calculate distance from the robot to the port
  2. use that distance and the predetermined speed to determine 
        a) the angle of the shooter

  vf^2 = vi^2 + 2ax 
  x = vt + at^2

  */

  public static double distance = 0;   // x distance from the port

  public static double height1 = 0;   // y height from the floor to the camera
  public static double height2 = 0;   // y height from the floor to the port height
  public static double angle1 = 0;   // mounting angle of the limelight
  public static double angle2 = 0;   // y angle to the target that limelight tells you

  public static double angleOfShooter = 0;   // angle of the hood of the shooter 
  
  public AutoLimelightShooter() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // Calculate distance from the robot to the power cell port
    distance = (height2-height1) / Math.tan(angle1 + angle2);


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
