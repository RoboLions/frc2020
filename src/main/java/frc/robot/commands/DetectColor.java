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
import com.revrobotics.getColor;
import com.revrobotics.ColorMatchResult;

public class DetectColor extends Command {
  
  private Port i2cport;

public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cport/*put port here*/);
  // ^^^ this line of code is an error because the port isn't here^ yet

  public int r;
  public int g;
  public int b;
  
  public String Color(int r, int g, int b) {

  String red = Color (255, 0, 255);
  String yellow = Color (255, 255, 0);
  String green = Color (0, 255, 0);
  String blue = Color (0, 255, 255);

  /*
  Color color = Color.red (255, 0, 255);
  Color color = Color.yellow (255, 255, 0);
  Color color = Color.green (0, 255, 0);
  Color color = Color.blue (0, 255, 255);
  ^^ this code didn't work because for some reason it says "Color cannot be resolved to a type"
  even though Color is imported
  */

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

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    String detectedColor = colorSensor.getColor();    
    /* ^^ getColor() gives the error:
    * "The method getColor() from the type ColorSensorV3 refers to the missing type Color"
    * Color is imported though?? (import edu.wpi.first.wpilibj.util.Color;)
    * It also says that the Color import isn't used even if it is
    */
  }

  @Override
  public void robotPeriodic() {
    colorSensor.getRawColor();
    
    String colorString;
    ColorMatchResult match = ColorMatch.matchClosestColor(detectedColor);

    if (match.Color = red) {
      colorString = "Red";
    }
    else if (match.Color = yellow) {
      colorString = "Yellow";
    }
    else if (match.Color = green) {
      colorString = "Green";
    }
    else if (match.Color = blue) {
      colorString = "Blue";
    }
    else {
      colorString = "Unknown";
    }
  }

  SmartDashboard.putString("Detected Color: ", colorString);

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