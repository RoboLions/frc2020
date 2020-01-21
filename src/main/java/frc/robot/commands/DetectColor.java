/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.ColorShim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C.Port;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class DetectColor extends Command {
  
  private Port i2cport;

public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cport/*put port here*/);
  // ^^^ this line of code is an error because the port isn't here^ yet

  public int r;
  public int g;
  public int b;
  
  public void Color(int r, int g, int b) {

  Color red = Color (255, 0, 255);
  Color yellow = Color (255, 255, 0);
  Color green = Color (0, 255, 0);
  Color blue = Color (0, 255, 255);

  }
  
  /*
  public final Color red = ColorMatch.ColorMake (225, 0, 255);
  public final Color yellow = ColorMatch.ColorMake (255, 255, 0);
  public final Color green = ColorMatch.ColorMake (0, 255, 0);
  public final Color blue = ColorMatch.ColorMake (0, 255, 255);
  */

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

 /*
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Color detectedColor = colorSensor.getColor();    
  }
  */

  @Override
  public void robotPeriodic() {
    colorSensor.getRawColor();
    
    String colorString;
    ColorMatchResult match = ColorMatch.matchClosestColor(detectedColor);

    if (match.Color = red) {
      
    }
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