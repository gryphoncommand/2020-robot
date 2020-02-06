package frc.lib.utils;

import com.revrobotics.CANSparkMax;

/**
 * Wrapper class to handle the overhead for the PID Controller that the CAN
 * SPARK MAX uses.
 */
public class PunkSparkMax extends CANSparkMax {
	/**
	 * Data type for SparkMax configs.
	 */
	public static class SparkMaxConfig {
		public double OPEN_LOOP_RAMP_RATE;
		public double CLOSED_LOOP_RAMP_RATE;
		public IdleMode IDLE_MODE;
		public boolean INVERTED;
	}

	public String name; // Specific name of device
	private SparkMaxConfig config;

	public PunkSparkMax(int deviceId, SparkMaxConfig _config) {
		super(deviceId, MotorType.kBrushless);
		config = _config;
		name = "Spark Max #" + deviceId;

		setConfig();
		restoreFactoryDefaults();
	}

	public PunkSparkMax(int deviceId, SparkMaxConfig _config, CANSparkMax leader) {
		this(deviceId, _config);
		follow(leader);
	}

	/**
	 * Set the configuration parameters to the given constants.
	 */
	private void setConfig() {
		super.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE);
		super.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE);
		super.setIdleMode(config.IDLE_MODE);
		super.setInverted(config.INVERTED);
	}

}
