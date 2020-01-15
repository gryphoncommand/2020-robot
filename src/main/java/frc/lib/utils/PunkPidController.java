package frc.lib.utils;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

public class PunkPidController extends CANPIDController {
	public static class PidControllerConfig {
		public double P;
		public double I;
		public double D;
		public CANPIDController.AccelStrategy ACCEL_STRATEGY;
		public double MAX_ACCEL;
		public double MIN_VELOCITY;
		public double MAX_VELOCITY;
	}

	public PunkPidController(CANSparkMax device, PidControllerConfig config){
		super(device);
	}
}
