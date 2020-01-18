/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/***  Written by Ava L. Byrd - 2019 FRC Season  ***/

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTableInstance;

/*************************************************************************
 * Subsystem that includes all the calling of Limelight values
 * 
 *************************************************************************/
public class Limelight extends Subsystem {
  
  //declare variables 
  //limelight network table
  public static NetworkTable limelightTable;

  //limelight x and y values for tracking coordinates
  public static double limelight_x;
  public static double limelight_y;

  //modes for limelight led light
  public static double FORCE_OFF = 1;
  public static double FORCE_BLINK = 2;
  public static double FORCE_ON = 3;

  //modes for limelight camera 
  public static double VISION_PROCESSOR = 0;
  public static double DRIVER_CAMERA = 1;

  /*************************************************************************
  * Call this under robot init at the beginning of execution for whole robot
  * 
  *************************************************************************/
  public static void init_Limelight() {
    //code for limelight vision and data
    
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry ty = limelightTable.getEntry("ty");

    limelight_x = (Double) tx.getDouble(0.0);
    limelight_y = (Double) ty.getDouble(0.0);

    //1 = force off, 2 = force blink, 3 = force on
    limelightTable.getEntry("ledMode").setNumber(FORCE_ON);
    //0 = Vision processor, 1 = Driver Camera
    limelightTable.getEntry("camMode").setNumber(VISION_PROCESSOR);
  }
  /*************************************************************************
  * Call this during a real time periodic thread to constantly refresh data
  * 
  *************************************************************************/
  public static void getLimelightData()  {
    limelight_x = (Double) limelightTable.getEntry("tx").getDouble(0.0);
    limelight_y = (Double) limelightTable.getEntry("ty").getDouble(0.0);
  }

  /*  public static void getLimelightData()  {
    limelight_x = (Double) limelightTable.getEntry("tx").getDouble(0.0);
    limelight_y = (Double) limelightTable.getEntry("ty").getDouble(0.0);
  }
  */
  
  /*************************************************************************
  * call this function to turn the led ON
  * 
  *************************************************************************/
  public static void turn_LED_ON()  {
    limelightTable.getEntry("ledMode").setNumber(FORCE_ON);
  }
  /*************************************************************************
  * call this function to turn the led OFF
  * 
  *************************************************************************/
  public static void turn_LED_OFF()  {
    limelightTable.getEntry("ledMode").setNumber(FORCE_OFF);
  }
  /*************************************************************************
  * call this function to cause the led to BLINK
  * 
  *************************************************************************/
  public static void turn_LED_FLASH_BLINK()  {
    limelightTable.getEntry("ledMode").setNumber(FORCE_BLINK);
  }
  /*************************************************************************
  * Use this to get the limelight x
  * 
  *************************************************************************/
  public static double getLimelightX() {
    return (Double) limelightTable.getEntry("tx").getDouble(0.0);
  }
  /*************************************************************************
  * Use this to get the limelight y
  * 
  *************************************************************************/
  public static double getLimelightY() {
    return (Double) limelightTable.getEntry("ty").getDouble(0.0);
  }
  /*************************************************************************
  * Use this to set the limelight to the regular driver camera mode
  * 
  *************************************************************************/
  public static void setDriverCamera() {
    limelightTable.getEntry("camMode").setNumber(DRIVER_CAMERA);
    turn_LED_OFF();
  }
  /*************************************************************************
  * Use this to set the limelight to vision processor mode
  * 
  *************************************************************************/
  public static void setVisionProcessor() {
    limelightTable.getEntry("camMode").setNumber(VISION_PROCESSOR);
    turn_LED_ON();
  }
  /*************************************************************************
  * Same as init_Limelight() but an older version
  * 
  *************************************************************************/
  public static void activateLimelight() {
    //code for limelight vision and data
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry ty = limelightTable.getEntry("ty");

    limelight_x = (Double) tx.getDouble(0.0);
    limelight_y = (Double) ty.getDouble(0.0);
    
    limelightTable.getEntry("ledMode").setNumber(FORCE_ON);
    limelightTable.getEntry("camMode").setNumber(VISION_PROCESSOR);
  }
  /*************************************************************************
  * get the network table object
  * 
  *************************************************************************/
  static NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
  
  }
  /*************************************************************************
  * Add your docs here.
  * 
  *************************************************************************/
  @Override
  public void initDefaultCommand() {

    
		//two of this command are present
	} //Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
}



// Limelight Tuning Notes TODO
/*
*   Increase distance from which you can use limelight = lower the range for percentage of image
*   Test blinders with index cards to limit view and add regular camera on top for driver camera instead of toggle
*   Fix laser
*
*
*
*/