package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
public class RobotMap {

    public static final int LEFT_FRONT_DRIVE_PORT = 1;
	public static final int LEFT_BACK_DRIVE_PORT = 3;
	public static final int RIGHT_FRONT_DRIVE_PORT = 2;
    public static final int RIGHT_BACK_DRIVE_PORT = 4;
    //public static final int ENCODER_MOTOR_PORT = #;

    
    public static WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(LEFT_FRONT_DRIVE_PORT);
    public static WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(RIGHT_FRONT_DRIVE_PORT);
    public static WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(LEFT_BACK_DRIVE_PORT);
    public static WPI_TalonSRX backRightMotor = new WPI_TalonSRX(RIGHT_BACK_DRIVE_PORT);

    //public static WPI_TalonSRX encoderMotor = new WPI_TalonSRX(ENCODER_MOTOR_PORT);

}