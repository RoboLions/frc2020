/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wiplibj.command.DetectColor;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.RobotMap;
import frc.robot.commands.RotationControl;

public class ColorWheel extends Subsystem {
  public static final int ColorCount = 0;

  public static void SmartDashboard() {
    SmartDashboard.putNumber("Encoder: Rotation", ColorWheel.getTicks());
  }
  
  public static double getTicks() {
    RotationControl.count = RobotMap.encoderMotor.get();
    return RotationControl.count;
  }

  public static double senseColor() {
    DetectColor.Color = RobotMap.encoderMotor.get();
    return DetectColor.Color;
  }

  /*public static final int color;
  public static final int red;

  public static void countColors() {
    color = getColorSensor();

    switch (color) {
      case 1:
        color = red;
        ColorCount++;
        break;
      case 2:
        color = blue;
        ColorCount++;
        break;
      case 3:
        color = yellow;
        ColorCount++;
        break;
      case 4:
        color = green;
        ColorCount++;
        break;
    }

    if (color == red) {
      ColorCount++;
    }
    else if (color == blue) {
      ColorCount++;
    }
    else if (color == green) {
      ColorCount++;
    }
    else if(color == yellow) {
      ColorCount++;
    }

}*/

  //This is for sending the color name to the SmartDashboard from the sensor
  //SmartDashboard.putString("Red", )
  //SmartDashboard.putString("Yellow", )
  //SmartDashboard.putString("Green", )
  //SmartDashboard.putString("Blue", )
  //SmartDashboard.putString("Unknown", )

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

