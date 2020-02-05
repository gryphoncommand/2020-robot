/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.utils.PunkSparkMax.SparkMaxConfig;
import frc.lib.utils.PunkPIDController.PidControllerConfig;
import com.revrobotics.CANSparkMax.IdleMode;

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
        public static final int kmLeftFront = 5;
        public static final int kmLeftBack = 6;
        public static final int kmRightFront = 3;
		public static final int kmRightBack = 4;
		public static final double kTrackWidth = 0.645668;
		public static final double kTankInputFactor = 0.25;
		public static final double kDistancePerPulse = 1.86e-5;
    }
	public static int[] kGearShift = {0, 1};
}
