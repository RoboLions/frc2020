/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class CountingSpins extends Command {
   //175.939188601 inches circumference of color wheel
    //?? (var: spin) inches circumference of spinner
    //175.939188601 divided by spin multiplied by 4 = total # times spinner needs to rotate
    //# times needed to rotate * # ticks per full rotation = total # ticks the encoder should
    //count before the spinner stops

  public static final double controlPanelCircumference = 175.939188601;
  public static final double spinerCircumference = 0.0; //placeholder 
  public static final double totalRotations = (175.939188601/0.0)*4;
  public static final double encoderTicksPR = 1440; //placeholder 
  public static final double totalEncoderTicks = totalRotations * encoderTicksPR;

  public CountingSpins() {
    //how to tell number of times spinner rotates: the encoder will tell
    //first reset the encoder to 0
    //then the encoder starts counting the number of ticks
    //when the encoder reaches totalEncoderTicks, the spinner will stop
    //how to reset an encoder?
    resetEncoder();

    //how to get the number of ticks the encoder is getting?
    //website will say # ticks per revolution of the motor
    public int getRaw() {
      int value;
      if (m_counter != null) {
        value = m_counter.get();
      } else {
        value = m_encoder.readOutput_Value();
      }
      return value;
    }

    if (value == totalEncoderTicks) {
      //spinner stops
    } else {

    }

  }

  private void resetEncoder() {
      //RobotMap.encoderMotor.getSensorCollection(0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
