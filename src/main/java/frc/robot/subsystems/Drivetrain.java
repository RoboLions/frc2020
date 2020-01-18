package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.commands.JoystickDrive; 
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import frc.robot.OI;
import frc.robot.lib.RoboLionsPID;
import frc.robot.Robot;
import com.ctre.phoenix.sensors.PigeonIMU;

import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;
import java.io.BufferedWriter;
import java.io.OutputStreamWriter;
import java.io.FileOutputStream;
import java.io.Writer;


public class Drivetrain extends Subsystem {
	public static boolean IS_ARCADE = true; // if it's not arcade, it's curvature
	
	public static final Joystick JOYSTICK = OI.getDriverJoystick();

    public int DRIVE_MODE = 0; // 0 = Voltage, 1 = PID mode, 2 = Current mode

    //a constant to make sure the limelight x value doesn't cause the motor power to be too high
    public static final double LLAIMINGCONSTANT = 0.035;
    //motor gain is a constant to make sure it dosn't put out too much power to the motors
    public static final double MOTORGAIN = 0.75; //0.8  //0.7; //0.6

    public static WPI_TalonSRX frontLeftMotor = RobotMap.frontLeftMotor;
    public static WPI_TalonSRX frontRightMotor = RobotMap.frontRightMotor;
    public static WPI_TalonSRX backLeftMotor = RobotMap.backLeftMotor;
    public static WPI_TalonSRX backRightMotor = RobotMap.backRightMotor;
    //WPI_TalonSRX centerMotor = RobotMap.centerMotor;

    SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

    public double effective_rate_command = 0.0; // m/s
    //public static final double DEFAULT_VOLTAGE_RAMP_RATE = 8.0;
    public static final double DEFAULT_VOLTAGE_RAMP_RATE = 100.0;
	// public static final int MOTOR_ENCODER_CODES_PER_REV = 1440; 2017
	public static final int MOTOR_ENCODER_CODES_PER_REV = 4096;
	//public static final int TIMEOUT_MS = 20; //20
	//public static final double MOTOR_ENCODER_CODES_PER_REV = 1440; // 2017
	//public static final int MOTOR_ENCODER_CODES_PER_REV = 4096;
	public static final int TIMEOUT_MS = 10; //20
	//public static final double WHEEL_DIAMETER_2017 = 0.10;
	//public static final double DIAMETER_INCHES = 7.15;// 2018 10 // bot 2 = 7.5
	public static final double DIAMETER_INCHES = 6.0;//2019 6 // 2018 10 // bot 2 = 7.5
	public static final double IN_TO_M = .0254;//.0254
	//public static final double WHEEL_DIAMETER_2018 = DIAMETER_INCHES * IN_TO_M; // in meters
	public static final double WHEEL_DIAMETER_2019 = DIAMETER_INCHES * IN_TO_M; // in meters
	//public static final double WHEEL_DIAMETER_2018 = ((DIAMETER_INCHES * .94) * IN_TO_M); // in meters
	public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_2019 * Math.PI;//3.14159;//(Math.PI;)
	//public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_2017 * Math.PI;//3.14159;//(Math.PI;)
	public static final double TICKS_PER_METER = (double)MOTOR_ENCODER_CODES_PER_REV * (double)(1.0/WHEEL_CIRCUMFERENCE);
			//1.0/0.59208918;
//			//1.6889348;
    public static final double M_SEC_TO_CNTS_100MSEC = (4096 / ((DIAMETER_INCHES * IN_TO_M) * 3.14) )*0.1;

	public static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
	public static final double BOT_WHEEL_TO_WHEEL_DIAMETER_INCHES = 23.7; //inches

	/**
	 * This translates bot angular speed to commands sent to the talons to make
	 * the robot run at that speed. 
	 */
	public static final double DEGREE_PER_SEC_TO_TICKS_PER_100ms = (BOT_WHEEL_TO_WHEEL_DIAMETER_INCHES * MOTOR_ENCODER_CODES_PER_REV) / 
								(360 * DIAMETER_INCHES * 10);
	
	
	// Ticks per meter should stay like this
	
	// public static final double MOTOR_NOMINAL_OUTPUT_VOLTAGE = 0.0;
	// public static final double MOTOR_PEAK_OUTPUT_VOLTAGE = 12.0;
	/******************************************************************** 
	 * 
	 * CHANGING GAIN TODO
	 * 
	*********************************************************************/
	public static final int MOTOR_PIDF_PROFILE = 0;
	public static final double MOTOR_GAIN_F = 1; //1
	public static final double MOTOR_GAIN_P = 1.2; // 1.2
	public static final double MOTOR_GAIN_I = 0;	
	public static final double MOTOR_GAIN_D = 0.3; //0.3
	//public static final double MOTOR_RAMPRATE = 0.5; //1.25
	public static final double MOTOR_RAMPRATE = 1; //1.25

	public static double leftVelocity = 0;
	public static double rightVelocity = 0;
	public static double leftPosition = 0;
	public static double rightPosition = 0;

	public double robotPosition = 0;
	public double robotVelocity = 0;

	public double yawGlobal = 0;
	public double rollGlobal = 0;
	public double pitchGlobal = 0;

	public double LX = Robot.limelight.limelight_x;

	WPI_TalonSRX leftMotorFront = RobotMap.frontLeftMotor;
	WPI_TalonSRX leftMotorBack = RobotMap.backLeftMotor;
	WPI_TalonSRX rightMotorFront = RobotMap.frontRightMotor;
	WPI_TalonSRX rightMotorBack = RobotMap.backRightMotor;
    PigeonIMU imu = new PigeonIMU(RobotMap.conveyerBeltMotor); 
    /***************************************
    WHY IS THIS ON LEFT INTAKE MOTOR??? 
    ***************************************/
	// code borrowed from standard wpi lib library       2/10/2018     M.F.
	DifferentialDrive robotDrive = RobotMap.robotDrive;
	
	public RoboLionsPID forwardPID = new RoboLionsPID();
	public RoboLionsPID headingPID = new RoboLionsPID();
	public RoboLionsPID limelightPID = new RoboLionsPID();
    
    public double rotate = 0;
    public double throttle = 0;

    public double ThetaLeftMotor = 0;
	public double ThetaRightMotor = 0;
	
	public double globalThrottle = 0;
	public double globalRotate = 0;

	public double leftTriggerPos = 0;
	public double rightTriggerPos = 0;

	public Writer writer;

	static boolean initRateDrive = true;

    public Drivetrain() {
				 
		try {
			writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream("test.txt"), "utf-8"));
		} catch (Exception e) {
			System.out.println("HELPPP");
		}

        //backLeftMotor.set(ControlMode.Follower, frontLeftMotor.getDeviceID());
		//backRightMotor.set(ControlMode.Follower, frontRightMotor.getDeviceID());
		
		frontLeftMotor.set(ControlMode.Follower, backLeftMotor.getDeviceID());
		frontRightMotor.set(ControlMode.Follower, backRightMotor.getDeviceID());
        
        frontLeftMotor.setSensorPhase(true);
		frontRightMotor.setSensorPhase(false);
		backLeftMotor.setSensorPhase(true);
		backRightMotor.setSensorPhase(false);
        
        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        backRightMotor.setInverted(true);
        backLeftMotor.setInverted(false);   

        leftMotorFront.configNominalOutputForward(0,TIMEOUT_MS);
        leftMotorFront.configNominalOutputReverse(0,TIMEOUT_MS);
        leftMotorFront.configPeakOutputForward(1,TIMEOUT_MS);
        leftMotorFront.configPeakOutputReverse(-1,TIMEOUT_MS);
        
        rightMotorFront.configNominalOutputForward(0,TIMEOUT_MS);
        rightMotorFront.configNominalOutputReverse(0,TIMEOUT_MS);
        rightMotorFront.configPeakOutputForward(1,TIMEOUT_MS);
        rightMotorFront.configPeakOutputReverse(-1,TIMEOUT_MS);
        
        leftMotorFront.configNeutralDeadband(0.001, TIMEOUT_MS);
        leftMotorBack.configNeutralDeadband(0.001, TIMEOUT_MS);
        rightMotorFront.configNeutralDeadband(0.001, TIMEOUT_MS);
		rightMotorBack.configNeutralDeadband(0.001, TIMEOUT_MS);
		
		/************************
		 * hey you, yah, you robolion, uncomment the below 
		 * code to enable coast mode
		leftMotorFront.setNeutralMode(NeutralMode.Coast);
		leftMotorBack.setNeutralMode(NeutralMode.Coast);
		rightMotorFront.setNeutralMode(NeutralMode.Coast);
		rightMotorBack.setNeutralMode(NeutralMode.Coast);
		************************/
		leftMotorFront.setNeutralMode(NeutralMode.Brake);
		leftMotorBack.setNeutralMode(NeutralMode.Brake);
		rightMotorFront.setNeutralMode(NeutralMode.Brake);
		rightMotorBack.setNeutralMode(NeutralMode.Brake);

        
        leftMotorFront.config_kF(0,MOTOR_GAIN_F,TIMEOUT_MS);
        leftMotorFront.config_kP(0,MOTOR_GAIN_P,TIMEOUT_MS);
        leftMotorFront.config_kI(0,MOTOR_GAIN_I,TIMEOUT_MS);
        leftMotorFront.config_kD(0,MOTOR_GAIN_D,TIMEOUT_MS);
        leftMotorFront.configClosedloopRamp(MOTOR_RAMPRATE, TIMEOUT_MS);
        
        rightMotorFront.config_kF(0,MOTOR_GAIN_F,TIMEOUT_MS);
        rightMotorFront.config_kP(0,MOTOR_GAIN_P,TIMEOUT_MS);
        rightMotorFront.config_kI(0,MOTOR_GAIN_I,TIMEOUT_MS);
        rightMotorFront.config_kD(0,MOTOR_GAIN_D,TIMEOUT_MS);
        rightMotorFront.configClosedloopRamp(MOTOR_RAMPRATE, TIMEOUT_MS);
        
        leftMotorBack.config_kF(0,MOTOR_GAIN_F,TIMEOUT_MS);
        leftMotorBack.config_kP(0,MOTOR_GAIN_P,TIMEOUT_MS);
        leftMotorBack.config_kI(0,MOTOR_GAIN_I,TIMEOUT_MS);
        leftMotorBack.config_kD(0,MOTOR_GAIN_D,TIMEOUT_MS);
        leftMotorBack.configClosedloopRamp(MOTOR_RAMPRATE, TIMEOUT_MS);
        
        rightMotorBack.config_kF(0,MOTOR_GAIN_F,TIMEOUT_MS);
        rightMotorBack.config_kP(0,MOTOR_GAIN_P,TIMEOUT_MS);
        rightMotorBack.config_kI(0,MOTOR_GAIN_I,TIMEOUT_MS);
        rightMotorBack.config_kD(0,MOTOR_GAIN_D,TIMEOUT_MS);
		rightMotorBack.configClosedloopRamp(MOTOR_RAMPRATE, TIMEOUT_MS);

		/***********************
		 * This is for Position control
		 */
		forwardPID.initialize(1, // Proportional Gain
                            0.1, // Integral Gain 0.0018
                            0, // Derivative Gain
                            0.1, // Cage Limit //0.3
                            0.01, // Deadband
                            1.0// MaxOutput //0.25
		);
		headingPID.initialize(2, // Proportional Gain //2
                            4, // Integral Gain //5
                            0, // Derivative Gain
                            10, // Cage Limit //10
                            0.1, // Deadband //0.1
                            90 // MaxOutput //180
		);
		limelightPID.initialize(1, // Proportional Gain
							0.0001, // Integral Gain
							0, // Derivative Gain
							40, // Cage Limit
							0.1, // Deadband
							50 // Max Output
		);
    }

    public void setModeVelocity() {
        frontLeftMotor.set(ControlMode.Velocity, 0);
        frontRightMotor.set(ControlMode.Velocity, 0);
    }

    public void setModeVoltage() {
        frontLeftMotor.set(ControlMode.PercentOutput, 0);
        frontRightMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setModeCurrent() {
        frontLeftMotor.set(ControlMode.Current, 0);
        frontRightMotor.set(ControlMode.Current, 0);
    }

    public void activateEncoders() {
        frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); //old encoder
        frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); //old encoder
    } 

    public void initDefaultCommand() {
    	setDefaultCommand(new JoystickDrive());
    }

    public double getYaw() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	// SmartDashboard.putNumber("IMU Yaw", ypr[0]);
		//System.out.println("yaw:" + ypr[0]);
		
		//Inverting the yaw to match a coordinate system with the z-axis pointing down
    	return -ypr[0];
    }
    
    public double getPitch() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	// SmartDashboard.putNumber("IMU Pitch", ypr[1]);
    	//System.out.println("pitch:" + ypr[1]);
    	return ypr[1];
    }
    
    public double getRoll() {
    	double[] ypr = new double[3];
    	imu.getYawPitchRoll(ypr);
    	// SmartDashboard.putNumber("IMU Roll", ypr[2]);
    	//System.out.println("roll:" + ypr[2]);
    	return ypr[2];
    }

    
    public double[] getRPH() {
    	double[] ypr = new double[3];
		imu.getYawPitchRoll(ypr);

		//Inverting yaw to match coordinate system with z-axis pointing down
		ypr[0] = -ypr[0];
    	// SmartDashboard.putNumber("IMU Yaw", ypr[0]);
    	// SmartDashboard.putNumber("IMU Pitch", ypr[1]);
    	// SmartDashboard.putNumber("IMU Roll", ypr[2]);
    	return(ypr);
    }
    
    double yaw = getRPH()[0];
    double pitch = getRPH()[1];
    double roll = getRPH()[2];
    
    public void ZeroYaw() {
    	imu.setYaw(0, TIMEOUT_MS);
    	imu.setFusedHeading(0, TIMEOUT_MS);
    }

    public double distanceTravelled() {
		return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
	}

	public double getLeftEncoderPosition() {
		// System.out.println(leftMotorFront.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("left position - ticks", leftMotorFront.getSensorCollection().getQuadraturePosition());
		return leftMotorFront.getSensorCollection().getQuadraturePosition();
	}

	public double getRightEncoderPosition() {
		// System.out.println(rightMotorFront.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("right position - ticks", rightMotorFront.getSensorCollection().getQuadraturePosition());
		return rightMotorFront.getSensorCollection().getQuadraturePosition();
	}
	
	public double getLeftEncoderVelocity() {
		// System.out.println(leftMotorFront.getSensorCollection().getQuadraturePosition());
	    // SmartDashboard.putNumber("left velocity - ticks", leftMotorFront.getSensorCollection().getQuadratureVelocity());
		//SmartDashboard.putNumber("left velocity - ticks/sec", leftMotorFront.getSelectedSensorVelocity(0));
		return leftMotorFront.getSensorCollection().getQuadratureVelocity();
	}

	public double getRightEncoderVelocity() {
		// System.out.println(rightMotorFront.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("right velocity - ticks", rightMotorFront.getSensorCollection().getQuadratureVelocity());
		//SmartDashboard.putNumber("right velocity - ticks/sec", rightMotorFront.getSelectedSensorVelocity(0));
		return rightMotorFront.getSensorCollection().getQuadratureVelocity();
	}
	
	// New library added to return speed in meters and meters per second
	

	public double distanceTravelledinMeters() {
		//double value = ((-1 * getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0) / TICKS_PER_METER;
		double value = -(0.5*((double)leftMotorFront.getSensorCollection().getQuadraturePosition() * METERS_PER_TICKS + 
				                    -(double)rightMotorFront.getSensorCollection().getQuadraturePosition() * METERS_PER_TICKS  )) ;
		// SmartDashboard.putString("Distance Traveled - meters", Double.toString(value));
		return value;
	}

	public double distanceTravelledinMeters2() {
		//double value = ((-1 * getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0) / TICKS_PER_METER;
		double value = -(0.5*((double)leftMotorFront.getSensorCollection().getQuadraturePosition() * METERS_PER_TICKS + 
				                    -(double)rightMotorFront.getSensorCollection().getQuadraturePosition() * METERS_PER_TICKS  )) ;
		// SmartDashboard.putString("Distance Traveled - meters", Double.toString(value));
		return value;
	}

	/*

	THIS CODE WAS WRITTEN ASSUMING THE ENCODERS WERE WIRED TO THE FRONT TALONS, WHEN IN REALITY THEY
	WERE WIRED TO THE BACK, AND THAT'S THE TEA

	public double getLeftEncoderPositioninMeters() {
		// System.out.println(leftMotorFront.getSensorCollection().getQuadraturePosition());
		double value = leftMotorFront.getSensorCollection().getQuadraturePosition() * METERS_PER_TICKS;
		// SmartDashboard.putNumber("left position - meters", value);
		return value;
	}

	public double getRightEncoderPositioninMeters() {
		double value = rightMotorFront.getSensorCollection().getQuadraturePosition() * METERS_PER_TICKS;
		// SmartDashboard.putNumber("right position - meters", value);
		return value;
	}
	
	public double getLeftEncoderVelocityinMetersperSec() {
		double value = (leftMotorFront.getSensorCollection().getQuadratureVelocity()) * METERS_PER_TICKS;
		// SmartDashboard.putNumber("left velocity - meters", -(leftMotorFront.getSensorCollection().getQuadratureVelocity()) * METERS_PER_TICKS);
		return value;
	}

	public double getRightEncoderVelocityinMetersperSec() {
		double value = (rightMotorFront.getSensorCollection().getQuadratureVelocity()) * METERS_PER_TICKS;
		// SmartDashboard.putNumber("right velocity - meters", (rightMotorFront.getSensorCollection().getQuadratureVelocity()) * METERS_PER_TICKS);
		return value;
	}
	
	public double VelocityinMeters() {
		double value = (-(leftMotorFront.getSensorCollection().getQuadratureVelocity()) + rightMotorFront.getSensorCollection().getQuadratureVelocity()) * METERS_PER_TICKS;
		// SmartDashboard.putString("Distance Traveled - meters", Double.toString(value));
		return value;
	}
	
	public double adjustMotors(double leftEncoder, double rightEncoder) {
		return -(leftEncoder/rightEncoder);
	}
	*/
	
	public void getTalonData() {
		/*
		leftVelocity = -1.0*getLeftEncoderVelocityinMetersperSec();
		rightVelocity = getRightEncoderVelocityinMetersperSec();
		leftPosition = -1.0*getLeftEncoderPositioninMeters();
		rightPosition = getRightEncoderPositioninMeters();
		*/
		//robotPosition = (leftPosition + rightPosition) * 0.5;
		robotPosition = distanceTravelledinMeters2();
		
		robotVelocity = (leftVelocity + rightVelocity) * 0.5;
	}

	public void getIMUData() {
		double[] tempRPY = getRPH();

		yawGlobal = tempRPY[0];
		pitchGlobal = tempRPY[1];
		rollGlobal = tempRPY[2];
	}

	public void getJoystickData() {
		globalThrottle = -JOYSTICK.getRawAxis(OI.AXIS_LEFT_STICK_Y);
		globalRotate = JOYSTICK.getRawAxis(OI.AXIS_RIGHT_STICK_X);
	}

	public void getTriggerData() {
		leftTriggerPos = JOYSTICK.getRawAxis(OI.AXIS_LEFT_TRIGGER);
		rightTriggerPos = JOYSTICK.getRawAxis(OI.AXIS_RIGHT_TRIGGER);
	}

	public void resetEncoders() {
		leftMotorFront.getSensorCollection().setQuadraturePosition(0, TIMEOUT_MS);
		rightMotorFront.getSensorCollection().setQuadraturePosition(0, TIMEOUT_MS);
	}

    public void variableDrive(double speed, double rotation, double shift) {
        if(IS_ARCADE) {
            robotDrive.arcadeDrive(speed, rotation);
        } else {
            boolean quickTurn = false; // will figure out how to get this var
            robotDrive.curvatureDrive(speed, rotation, quickTurn);
        }
        //centerMotor.set(shift);
    }
	
	/***********************************************
	 * 23MAR19, IGY, this is the desired method of 
	 * controlling the bot by the drivers as of 23MAR19
	 * 
	 * hard to drive straight for closer/tighter shifts - NL 3/23/19
	 * @param throttle
	 * @param rotate
	 * @param sens
	 * @param tank
	 */
    public void setPower(double throttle, double rotate, double sens, boolean tank) { //sens is sensitivity, tank is whether it is in arcade drive mode or not
		
		initRateDrive = true;

		rotate = (Math.tan(.44 * (rotate * Math.PI))) / 5; //these are changeable curves that create the exact speed over time curve wanted
		throttle = (Math.tan(.42 * (throttle * Math.PI))) / 4;//same as above ^

		double ang = 0;
		double turnS = sens;

		double left_power = 0.0;
		double right_power = 0.0;
		
		ang = rotate * turnS;
		
		left_power = throttle + ang;
		right_power = throttle - ang;
		
		if(left_power > 1.0) {
			left_power = 1;
		}
		if(left_power < -1.0) {
			left_power = -1;
		}
		if(right_power > 1.0) {
			right_power = 1;
		}
		if(right_power < -1.0) {
			right_power = -1;
		}
		
        
            //this section cuts off power to the talons as a deadband 
            //input if there is no motion from the joystick 	
		if(Math.abs(throttle)<.1 && Math.abs(rotate)<.1){
		
			frontLeftMotor.set(.01);
			frontRightMotor.set(.01);
			backLeftMotor.set(.01);
			backRightMotor.set(.01);

        } 	else {

			/************************************
				Limelight Set Power Aiming Code
			 ************************************/
            if (OI.getDriverJoystick().getRawButton(OI.BUTTON_X))   {
                //if (Limelight.getLimelightX() > 0) {
                if (LX > 0) {
                    //frontLeftMotor.set(left_power * Limelight.getLimelightX() * llAimingConstant * .6);
                    frontLeftMotor.set(left_power * MOTORGAIN + LX * LLAIMINGCONSTANT);
                    frontRightMotor.set(right_power * MOTORGAIN);
                    //backLeftMotor.set(left_power * Limelight.getLimelightX() * llAimingConstant * .6);
                    backLeftMotor.set(left_power * MOTORGAIN + LX * LLAIMINGCONSTANT);
                    backRightMotor.set(right_power * MOTORGAIN);
                }

                //if (Limelight.getLimelightX() < 0) {
                if (LX < 0) {
                    frontLeftMotor.set(left_power * MOTORGAIN);
                    //frontRightMotor.set(right_power * -Limelight.getLimelightX() * llAimingConstant * .6);
                    // LX is negative because the power going to the talons is negative
                    frontRightMotor.set(right_power * MOTORGAIN - LX * LLAIMINGCONSTANT);
                    backLeftMotor.set(left_power * MOTORGAIN);
                    //backRightMotor.set(right_power * -Limelight.getLimelightX() * llAimingConstant * .6);
                    backRightMotor.set(right_power * MOTORGAIN - LX * LLAIMINGCONSTANT);
                } 
            } else {
                frontLeftMotor.set(left_power * MOTORGAIN);
                frontRightMotor.set(right_power * MOTORGAIN);
                backLeftMotor.set(left_power * MOTORGAIN);
				backRightMotor.set(right_power * MOTORGAIN);

				String leftvel = Double.toString(backLeftMotor.getSensorCollection().getQuadratureVelocity() * METERS_PER_TICKS);
				String rightvel = Double.toString(backRightMotor.getSensorCollection().getQuadratureVelocity() * METERS_PER_TICKS);
				String thstring = Double.toString(throttle);

				System.out.println(thstring + "   |   " + leftvel + "   |   " + rightvel);

			}
        }
     }

     // FOR USE IN TELEOP
     public void rateDriveImproved(double th,//velocity throttle 
			                double ro, //rotation from joystick
							double off, double speed_limit) {//heading offset
		
		if(initRateDrive) {
			frontLeftMotor.set(ControlMode.Follower, backLeftMotor.getDeviceID());
			frontRightMotor.set(ControlMode.Follower, backRightMotor.getDeviceID());
			initRateDrive = false;
		}
		
		ro=0;//ro = (Math.tan(.46 * (ro * Math.PI))) / 6; //these are changeable curves that create the exact speed over time curve wanted
		th = (Math.tan(.47 * (th * Math.PI))) / 3;//same as above ^
		
		if (th > speed_limit) {
			th = speed_limit;
		}
		if (th < -speed_limit) {
			th = -speed_limit;
		}
		double linearSpeedPer100ms = th * TICKS_PER_METER * 0.1; //METERS_PER_TICKS * 0.01;
		// transforms meter per second to ticks per 100 milliseconds
		double Kpivot = 360;
		double ThetaCMD = linearSpeedPer100ms * ro * Kpivot;
	
		//double target_speed = TICKS_PER_METER / 10;
		//double Kthrottle = 1.0;
		//double Kpivot = 500.0;
		//double ThetaCMD = Kthrottle * th * target_speed;	
		double Thetaheading = (Kpivot * ro) * DEGREE_PER_SEC_TO_TICKS_PER_100ms;// - off;

		ThetaLeftMotor = linearSpeedPer100ms + (Thetaheading + off);
		ThetaRightMotor = linearSpeedPer100ms - (Thetaheading + off);
		// ThetaLeftMotor = ro;
		// ThetaRightMotor = -ro;

		String leftvel = Double.toString(backLeftMotor.getSensorCollection().getQuadratureVelocity() * METERS_PER_TICKS);
		String rightvel = Double.toString(backRightMotor.getSensorCollection().getQuadratureVelocity() * METERS_PER_TICKS);
		String thstring = Double.toString(th);

		System.out.println(thstring + "   |   " + leftvel + "   |   " + rightvel);

		// 1500 RPM in either direction 
		//System.out.println("throttle: " + th);
		backLeftMotor.set(ControlMode.Velocity, ThetaLeftMotor);
		//System.out.println("left velocity: " + leftMotorFront.getSelectedSensorVelocity(0));
		backRightMotor.set(ControlMode.Velocity, ThetaRightMotor);
		//System.out.println("right velocity: " + rightMotorFront.getSelectedSensorVelocity(0));
		if (OI.getDriverJoystick().getRawButton(OI.BUTTON_X)) {
			backLeftMotor.set(ControlMode.Velocity, ThetaLeftMotor);
			backRightMotor.set(ControlMode.Velocity, ThetaRightMotor);

			/****************************************
			  Limelight Rate Drive Code
			 ****************************************/
			if (LX > 0) {
				backLeftMotor.set(ThetaLeftMotor + LX*LLAIMINGCONSTANT);
				backRightMotor.set(ThetaRightMotor);
			}

			//if (Limelight.getLimelightX() < 0) {
			if (LX < 0) {
				backLeftMotor.set(ThetaLeftMotor);
				backRightMotor.set(ThetaRightMotor + LX*LLAIMINGCONSTANT);
			}
		}
		else {
			backLeftMotor.set(ControlMode.Velocity, ThetaLeftMotor);
			//System.out.println("left velocity: " + leftMotorFront.getSelectedSensorVelocity(0));
			backRightMotor.set(ControlMode.Velocity, ThetaRightMotor);
			//System.out.println("right velocity: " + rightMotorFront.getSelectedSensorVelocity(0));
		}
	}

	public void rateDrive(double th,//velocity throttle 
			                double ro, //rotation from joystick
			                double off, double speed_limit) {//heading offset
		
		// 2018 test code to perfect motion control
		// Uses Pigeon IMU and Closed Loop Velocity
		// Checkable in web based configuration
		// Web address - 10.12.61.2
		

		double motorOutputLeft = leftMotorFront.getMotorOutputPercent();
		double motorOutputRight = rightMotorFront.getMotorOutputPercent();
		
		/*
		sb.append("\toutleft:");
		sb.append(motorOutputLeft);
		sb.append("\toutright:");
		sb.append(motorOutputRight);
		sb.append("\tspdleft:");
		sb.append(leftMotorFront.getSelectedSensorVelocity(0));
		sb.append("\tspdright:");
		sb.append(rightMotorFront.getSelectedSensorVelocity(0));
       */

		double throttle_scaler = 2;
		
		//th = th * 1077;
	
		th = throttle_scaler * th; // to meters per second
		
		// take input throttle from joystick and transform 
		// meters per second --> encoder counts per 100 milliseconds
		
		//double speed_limit = .5; // 1
//>>>>>>> parent of b4351bc... ASHEVILLE DAY 0 CHANGES
		
		if (th > speed_limit) {
			th = speed_limit;
		}
		if (th < -speed_limit) {
			th = -speed_limit;
		}
		/*
		if (ro > speed_limit) {
			ro = speed_limit;
		}
		if (ro < -speed_limit) {
			ro = -speed_limit;
		}
		*/
		// circumference of wheel = 4 * pi
		// circumference of COMPETITION wheel = 8 * pi
			double mps_to_encp100ms = TICKS_PER_METER / 10; //METERS_PER_TICKS * 0.01;
			// transforms meter per second to ticks per 100 milliseconds
			double Kpivot = 400;
			double ThetaCMD = mps_to_encp100ms * th;
	
			//double target_speed = TICKS_PER_METER / 10;
			//double Kthrottle = 1.0;
			//double Kpivot = 500.0;
			//double ThetaCMD = Kthrottle * th * target_speed;	
			double Thetaheading = (Kpivot * ro);// - off;
			//System.out.println(off);
			//double Koffset = 1; 
			double Thetaoffset = Thetaheading;
			ThetaLeftMotor = ThetaCMD + Thetaoffset;
			ThetaRightMotor = ThetaCMD - Thetaoffset;
			// ThetaLeftMotor = ro;
			// ThetaRightMotor = -ro;
		 
		// 1500 RPM in either direction 
		//System.out.println("throttle: " + th);
		leftMotorFront.set(ControlMode.Velocity, ThetaLeftMotor);
		//System.out.println("left velocity: " + leftMotorFront.getSelectedSensorVelocity(0));
		rightMotorFront.set(ControlMode.Velocity, ThetaRightMotor);
		//System.out.println("right velocity: " + rightMotorFront.getSelectedSensorVelocity(0));
	}

	//FOR USE IN AUTONOMOUS
	public void autoRateDrive(double speed,//velocity throttle (in meters per second)
							double rotationalSpeed, //rotation command (in degrees per second)
							double speed_limit) {
		
		// 2018 test code to perfect motion control
		// Uses Pigeon IMU and Closed Loop Velocity
		// Checkable in web based configuration
		// Web address - 10.12.61.2

		
		if (speed > speed_limit) {
			speed = speed_limit;
		}
		if (speed < -speed_limit) {
			speed = -speed_limit;
		}

		/**
		 * This is the native command that is sent to the Talon; converts tangential speed of the robot into encoder ticks
		 */
		double speedInTicksPer100ms = speed * TICKS_PER_METER * 0.10; //10 is seconds to 100 ms
		
		double rotationalSpeedInTicksPer100ms = rotationalSpeed * DEGREE_PER_SEC_TO_TICKS_PER_100ms;
		
		ThetaLeftMotor = speedInTicksPer100ms + rotationalSpeedInTicksPer100ms;
		ThetaRightMotor = speedInTicksPer100ms - rotationalSpeedInTicksPer100ms;
		 

		System.out.println("Yaw Data: " + speedInTicksPer100ms);
		leftMotorFront.set(ControlMode.Velocity, ThetaLeftMotor);
		rightMotorFront.set(ControlMode.Velocity, ThetaRightMotor);
	}


    public void coordinateDrive(double x, double y, double rotate) {
        if(x != 0) {
            //centerMotor.set(ControlMode.Position, x);
        } else {
            if(y != 0) {
                frontLeftMotor.set(ControlMode.Position, y);
                frontRightMotor.set(ControlMode.Position, y);
            }
            if(rotate > 0) {
                frontLeftMotor.set(ControlMode.Position, rotate);
                frontRightMotor.set(ControlMode.Position, -rotate);
            } else if(rotate < 0) {
                frontLeftMotor.set(ControlMode.Position, -rotate);
                frontRightMotor.set(ControlMode.Position, rotate);
            }
        }
    }

    // magnitude is from 0 to 1, direction is out of 360 degrees
    public void vectorDrive(double magnitude, double direction) {
        double forward = magnitude * Math.sin(direction * (Math.PI/180));
        double sideways = magnitude * Math.cos(direction * (Math.PI/180));
        frontLeftMotor.set(forward);
        frontRightMotor.set(forward);
        //centerMotor.set(sideways);
    }
    public void stop() {
        frontLeftMotor.set(0.001);
        frontRightMotor.set(0.001);
    }
}
