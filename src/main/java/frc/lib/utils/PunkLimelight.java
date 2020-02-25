package frc.lib.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wrapper class to handle the Limelight.
 */
public class PunkLimelight {
	/**
	 * Data type for Limelight configs.
	 */
	public static class LimelightConfig {
		public int LED_BEHAVIOR;
		public int CAMERA_TYPE;
		public int PIPELINE;
		public int SNAPSHOT;
	}

	public NetworkTable table; // 'Limelight' table
	public NetworkTableEntry tx, ty, ta, tv; // Output entries.

	private LimelightConfig config;

	public PunkLimelight(int deviceId, LimelightConfig _config) {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
		tv = table.getEntry("tv");
		config = _config;
		setConfig();
	}

	/**
	 * Set the configuration parameters to the given constants.
	 */
	private void setConfig() {
		table.getEntry("ledMode").setNumber(config.LED_BEHAVIOR);
		table.getEntry("camMode").setNumber(config.CAMERA_TYPE);
		table.getEntry("steam").setNumber(config.PIPELINE);
		table.getEntry("snapshot").setNumber(config.SNAPSHOT);
	}

	/**
	 * Returns the top target that the Limelight gets.
	 * 
	 * @return Array of target's x-position, y-position, and area.
	 */
	public double[] getTarget() {
		if (tv.getBoolean(false)) {
			double[] target = { tx.getDouble(0), ty.getDouble(0), ta.getDouble(0) };
			return (target);
		}
		return null;
	}

}
