package frc.lib.utils;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Wrapper class to handle the overhead for the PID Controller that the CAN
 * SPARK MAX uses.
 * 
 * TODO: Add a setReference() wrapper to reduce the overhead (Unit conversions?)
 * Add a ControlType to Config? (It is fluid so it may not be necessary.)
 */
public class PunkPIDController extends CANPIDController {

	/**
	 * Data type for Controller configs.
	 */
	public static class PIDControllerConfig {
		public double P;
		public double I;
		public double D;
		public double MIN_OUTPUT;
		public double MAX_OUTPUT;
	}

	private NetworkTableEntry pEntry, iEntry, dEntry; // Shuffleboard objects
	private PIDControllerConfig config; // Local instance of configs to be used internally.

	public PunkPIDController(PunkSparkMax device, PIDControllerConfig _config, boolean isDebug) {
		super(device);
		config = _config;
		setConfig();
		if (isDebug) {
			dumpData(device.name);
		}
	}

	/**
	 * Push the PID constants to a ShuffleBoard List.
	 */
	private void dumpData(String name) {
		pEntry = Shuffleboard.getTab("Robot Main").getLayout(name).add("P", config.P).getEntry();
		iEntry = Shuffleboard.getTab("Robot Main").getLayout(name).add("I", config.I).getEntry();
		dEntry = Shuffleboard.getTab("Robot Main").getLayout(name).add("D", config.D).getEntry();
	}

	/**
	 * Check and update the PID constants to enable for dynamic tuning.
	 */
	public void updateData() {
		config.P = pEntry.getDouble(0.0);
		config.I = iEntry.getDouble(0.0);
		config.D = dEntry.getDouble(0.0);
		setConfig();
	}

	/**
	 * Set the configuration parameters to the given constants.
	 */
	public void setConfig() {
		super.setP(config.P);
		super.setI(config.I);
		super.setD(config.D);
		super.setOutputRange(config.MIN_OUTPUT, config.MAX_OUTPUT);
	}
}
