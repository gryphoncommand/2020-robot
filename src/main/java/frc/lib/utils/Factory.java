/*
/ Helper class for returning "prefab" motors, sensors, etc.
*/
package frc.lib.utils;

import com.revrobotics.CANPIDController;

public class Factory {
	public static class SparkMaxConfig {
		public int OPEN_LOOP_RAMP_RATE;
		public int CLOSED_LOOP_RAMP_RATE;
	}

	public static class PidControllerConfig {
		public double P;
		public double I;
		public double D;
		public CANPIDController.AccelStrategy ACCEL_STRATEGY;
		public double MAX_ACCEL;
		public double MIN_VELOCITY;
		public double MAX_VELOCITY;
	}
}
