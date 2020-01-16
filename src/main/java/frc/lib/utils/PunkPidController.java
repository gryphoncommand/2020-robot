package frc.lib.utils;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

public class PunkPidController extends CANPIDController {
	public static class PidControllerConfig {
		public double P;
		public double I;
		public double D;
		public double MAX_ACCEL;
		public double MIN_VELOCITY;
		public double MAX_VELOCITY;
	}

	public PunkPidController(CANSparkMax device, PidControllerConfig config){
		super(device);
		setConfig(config);
	}

	public void setConfig(PidControllerConfig config) {
		super.setP(config.P);
		super.setI(config.I);
		super.setD(config.D);
		super.setSmartMotionMaxAccel(config.MAX_ACCEL, 0);
		super.setSmartMotionMinOutputVelocity(config.MIN_VELOCITY, 0);
		super.setSmartMotionMaxVelocity(config.MIN_VELOCITY, 0);
	}
}
