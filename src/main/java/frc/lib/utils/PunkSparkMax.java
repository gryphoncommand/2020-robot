package frc.lib.utils;

import com.revrobotics.CANSparkMax;

public class PunkSparkMax extends CANSparkMax {
	public static class SparkMaxConfig {
		public double OPEN_LOOP_RAMP_RATE;
		public double CLOSED_LOOP_RAMP_RATE;
	}

	public PunkSparkMax(int deviceId, SparkMaxConfig config) {
		super(deviceId, MotorType.kBrushless);
		setConfig(config);
	}

	public PunkSparkMax(int deviceId, SparkMaxConfig config, CANSparkMax leader) {
		super(deviceId, MotorType.kBrushless);
		setConfig(config);
		follow(leader);
	}

	public void setConfig(SparkMaxConfig config) {
		super.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE);
		super.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE);
	}

}
