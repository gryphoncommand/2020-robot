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

	public PunkSparkMax(int deviceId, SparkMaxConfig config) {
		super(deviceId, MotorType.kBrushless);
		setConfig(config);
		name = "Spark Max #" + deviceId;
		restoreFactoryDefaults();
	}

	public PunkSparkMax(int deviceId, SparkMaxConfig config, CANSparkMax leader) {
		this(deviceId, config);
		follow(leader);
	}

	/**
	 * Set the configuration parameters to the given constants.
	 */
	public void setConfig(SparkMaxConfig config) {
		super.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE);
		super.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE);
		super.setIdleMode(config.IDLE_MODE);
		super.setInverted(config.INVERTED);
	}

}
