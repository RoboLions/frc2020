package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


public class Climber extends Subsystem {

    public static final double IN_POWER = 0;
    public static final double OUT_POWER = 0;
    public static final double STOP_POWER = 0;

    public static double encoderTicksTotal = 0;
    public static final double encoderTicksPerRev = 2000; // placeholder
    public static final double maxRotationTicks = 8000; // placeholder

	public WPI_TalonSRX climberMotor = RobotMap.climberMotor;

    public Climber() {
        climberMotor.setInverted(false);
    }

    public void winchClimbUp() {
        if (encoderTicksTotal < maxRotationTicks) {
            climberMotor.set(IN_POWER);
        }
        else {
           stopCLimber();
        }
    }

    public void winchClimbDown() {
        climberMotor.set(OUT_POWER);
    }

    public void stopCLimber() {
        climberMotor.set(STOP_POWER);
    }

    @Override
    protected void initDefaultCommand() {

    }

}