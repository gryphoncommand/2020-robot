/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.utils.PunkSparkMax.SparkMaxConfig;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.lib.utils.PunkLimelight.LimelightConfig;
import frc.lib.utils.PunkPIDController.PIDControllerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final SparkMaxConfig kDriveMotorConfig = new SparkMaxConfig();
	static {
		kDriveMotorConfig.OPEN_LOOP_RAMP_RATE = 0.0;
		kDriveMotorConfig.CLOSED_LOOP_RAMP_RATE = 0.0;
		kDriveMotorConfig.IDLE_MODE = IdleMode.kCoast;
		kDriveMotorConfig.INVERTED = false;
	}

	public static final class Joystick {
		public static final int kLeftXAxis = 0;
		public static final int kLeftYAxis = 1;
		public static final int kRightXAxis = 2;
		public static final int kRightYAxis = 3;
	}

	public static final class Drivetrain {
		public static final int kmLeftFront = 1;
		public static final int kmLeftBack = 2;
		public static final int kmRightFront = 3;
		public static final int kmRightBack = 4;

		public static final double kTrackWidth = 0.645668;
		public static final double kTankInputFactor = 0.25;
		public static final double kDistancePerPulse = 1.86e-5;

		public static final SparkMaxConfig kConfig = new SparkMaxConfig();
		static {
			kDriveMotorConfig.OPEN_LOOP_RAMP_RATE = 0.0;
			kDriveMotorConfig.CLOSED_LOOP_RAMP_RATE = 0.0;
			kDriveMotorConfig.IDLE_MODE = IdleMode.kCoast;
			kDriveMotorConfig.INVERTED = false;
		}
		public static final PIDControllerConfig kPIDConfig = new PIDControllerConfig();
		static {
			kPIDConfig.P = 0.1;
			kPIDConfig.I = 1e-4;
			kPIDConfig.D = 1;
			kPIDConfig.MIN_OUTPUT = -1;
			kPIDConfig.MAX_OUTPUT = 1;
		}
	}
	public static final class Intake {
		public static final int kmIntake = 0;
		public static final int kmIndexer = 1;
	}
	public static final class Shooter {
		public static final int kmShooter = 5;
		public static final SparkMaxConfig kConfig = new SparkMaxConfig();
		static {
			kConfig.OPEN_LOOP_RAMP_RATE = 0.0;
			kConfig.CLOSED_LOOP_RAMP_RATE = 0.0;
			kConfig.IDLE_MODE = IdleMode.kCoast;
			kConfig.INVERTED = false;
		}
	}
	public static final LimelightConfig kLimelight = new LimelightConfig();
	static {
		kLimelight.LED_BEHAVIOR = 0;
		kLimelight.CAMERA_TYPE = 1;
		kLimelight.PIPELINE = 0;
		kLimelight.SNAPSHOT = 0;
	}
	public static final class Limelight {
		// Target height (reflective tape on the bottom of center hexagon)
		public static final double kTargetHeight = 2.1145;
		// Mount height of Limelight
		public static final double kMountHeight = 0.0;
		// Mount angle of Limelight
		public static final double kMountAngle = 0.0;
		// Optimal firing distance 
		public static final double kOptimalTargetDistance = 1.3716;
		// Proportional distance constant.
		public static final double kPDistance = 0.0;
		// Proportional angle constant.
		public static final double kPAngle = 0.0;
	}

	public static int[] kGearShift = { 0, 1 };

}
