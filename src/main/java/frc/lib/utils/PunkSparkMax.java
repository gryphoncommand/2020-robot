package frc.lib.utils;

import com.revrobotics.CANSparkMax;

public class PunkSparkMax extends CANSparkMax {
	public class SparkMaxConfig {
		public double OPEN_LOOP_RAMP_RATE;
		public double CLOSED_LOOP_RAMP_RATE;
	}

	public PunkSparkMax(int deviceId, SparkMaxConfig config) {
		super(deviceId, MotorType.kBrushless);
	}

	public PunkSparkMax(int deviceId, SparkMaxConfig config, CANSparkMax master) {
		super(deviceId, MotorType.kBrushless);
		follow(master);
	}

}
