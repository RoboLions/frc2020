/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//the errors that relate to edu.wpi.frist.wpilibj.util.Color; work in the 2020 version
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class DetectColor extends Command {

  //private Port i2cport;
  public final I2C.Port i2cPort = I2C.Port.kOnboard; //change the port to match the connection of sensor

  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort/* put port here */);
  // ^^^ this line of code is an error because the port isn't here^ yet

  public final ColorMatch m_colorMatcher = new ColorMatch();

//public Color *color name* = new Color (double red, double green, double blue)
public static final Color red = new Color(1, 0, 1);
public static final Color yellow = new Color(1, 1, 0);
public static final Color green = new Color(0, 1, 0);
public static final Color blue = new Color(0, 1, 1);

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_colorMatcher.addColorMatch(red);
    m_colorMatcher.addColorMatch(yellow);
    m_colorMatcher.addColorMatch(green);
    m_colorMatcher.addColorMatch(blue);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    final Color detectedColor = colorSensor.getColor();
    //colorSensor.getColor(); 
    final String colorString = new String();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == red) {
      colorString = "Red";
    }
    else if (match.color == yellow) {
      colorString = "Yellow";
    }
    else if (match.color == green) {
      colorString = "Green";
    }
    else if (match.color == blue) {
      colorString = "Blue";
    }
    else {
      colorString = "Unknown";
    }
  
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Yellow", detectedColor.yellow);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
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