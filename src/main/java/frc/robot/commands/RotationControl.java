/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

public class RotationControl extends Command {
  public static final double controlPanelCircumference = 100.530964915; //pi*diameter
  public static final double spinnerCircumference = 0.0; //placeholder 
  public static final double totalRotations = (controlPanelCircumference/spinnerCircumference)*4;
  public static final double encoderTicksPR = 1440; //placeholder 
  public static final double totalEncoderTicks = totalRotations * encoderTicksPR;
  public static double count; //individual encoder ticks

  //Encoder sampleEncoder = new Encoder(0, 0); //placeholder (is this where this should be?)

  public static void resetEncoder() {
    count = 0;
  }

  public static void CountingSpins() {
    //first reset the encoder to 0
    //then the encoder starts counting the number of ticks
    //when the encoder reaches totalEncoderTicks, the spinner will stop
    resetEncoder();

    if (count<=(totalEncoderTicks-100)) {
      //continue spinning motor
      RobotMap.encoderMotor.set(0.3);
    } else if (count<=totalEncoderTicks) {
      //slow the motor
      RobotMap.encoderMotor.set(0.2);
    }
    else {
      //stop the motor
      RobotMap.encoderMotor.set(0);
    }
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
