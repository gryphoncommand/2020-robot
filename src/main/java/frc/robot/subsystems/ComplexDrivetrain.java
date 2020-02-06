package frc.robot.subsystems;

import frc.lib.utils.PunkSparkMax;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;

import io.github.oblarg.oblog.Loggable;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("PMD.ExcessiveImports")
public class ComplexDrivetrain extends SubsystemBase implements Loggable {
	private PunkSparkMax m_leftFront, m_leftBack, m_rightFront, m_rightBack;

	private SpeedControllerGroup m_left, m_right;

	private final DifferentialDrive m_drive;
	private final DifferentialDriveKinematics m_kinematics;
	private final DifferentialDriveOdometry m_odometry;

	private final PIDController m_leftPIDController, m_rightPIDController;

	private DoubleSolenoid m_gearShift;

	private Encoder m_leftEncoder;

	private Encoder m_rightEncoder;
	private CANEncoder m_testEncoder;

	private PigeonIMU m_gyro;

	private double[] heading;

	/**
	 * Central class for all drivetrain-related activities.
	 */
	public ComplexDrivetrain() {
		m_leftFront = new PunkSparkMax(Constants.Drivetrain.kmLeftFront, Constants.Drivetrain.kConfig);
		m_leftBack = new PunkSparkMax(Constants.Drivetrain.kmLeftBack, Constants.Drivetrain.kConfig, m_leftFront);
		m_rightFront = new PunkSparkMax(Constants.Drivetrain.kmRightFront, Constants.Drivetrain.kConfig);
		m_rightBack = new PunkSparkMax(Constants.Drivetrain.kmRightBack, Constants.Drivetrain.kConfig, m_rightFront);

		m_left = new SpeedControllerGroup(m_leftFront, m_leftBack);
		m_right = new SpeedControllerGroup(m_rightFront, m_rightBack);

		m_gyro = new PigeonIMU(0);
		heading = new double[3];

		m_drive = new DifferentialDrive(m_left, m_right);
		m_kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.kTrackWidth);
		m_odometry = new DifferentialDriveOdometry(getAngle());

		m_leftPIDController = new PIDController(1, 0, 0);
		m_rightPIDController = new PIDController(1, 0, 0);

		m_gearShift = new DoubleSolenoid(Constants.kGearShift[0], Constants.kGearShift[1]);
		m_leftEncoder = new Encoder(0, 1, 2);
		m_rightEncoder = new Encoder(3, 4, 5);

		m_testEncoder = m_leftFront.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
		SmartDashboard.putNumber("Drivetrain - Test Encoder", m_testEncoder.getPosition());

		m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.Drivetrain.kDistancePerPulse);
		m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.Drivetrain.kDistancePerPulse);
		m_leftEncoder.reset();
		m_rightEncoder.reset();
		m_gyro.configFactoryDefault();

		SmartDashboard.putData("Drivetrain - Left PID", m_leftPIDController);
		SmartDashboard.putData("Drivetrain - Right PID", m_rightPIDController);
		SmartDashboard.putData("Drivetrain - Left Encoder", m_leftEncoder);
		SmartDashboard.putData("Drivetrain - Right Encoder", m_rightEncoder);
	}

	/**
	 * Basic drivetrain operation, with no feed-forward control.
	 * 
	 * @param lSpeed Left Speed of Drivetrain
	 * @param rSpeed Right Speed of Drivetrain
	 */
	public void tankDrive(double lSpeed, double rSpeed) {
		lSpeed = Constants.Drivetrain.kTankInputFactor * lSpeed;
		rSpeed = Constants.Drivetrain.kTankInputFactor * rSpeed;
		m_left.set(lSpeed);
		m_right.set(rSpeed);
	}

	/**
	 * Basic drivetrain operation, with no feed-forward control.
	 * 
	 * @param xSpeed    Speed along the x-axis (Forward is positive)
	 * @param zRotation Rotation rate for robot (Clockwise is positive)
	 */
	public void curvatureDrive(double xSpeed, double zRotation) {
		m_drive.curvatureDrive(xSpeed, zRotation, false);
	}

	/**
	 * PID-Controlled drivetrain, with kinematics and odometry to be more precise.
	 * 
	 * @param _speeds Individual speeds of each side.
	 */
	public void pidTest(DifferentialDriveWheelSpeeds _speeds) {
		final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), _speeds.leftMetersPerSecond);
		final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(),
				_speeds.rightMetersPerSecond);
		m_leftFront.setVoltage(leftOutput);
		m_rightFront.setVoltage(rightOutput);
	}

	/**
	 * PID-Controlled drivetrain, but without kinematics or odometry.
	 * 
	 * @param lSpeed Left Speed of Drivetrain
	 * @param rSpeed Right Speed of Drivetrain
	 */
	public void pidTankDrive(double lSpeed, double rSpeed) {
		updateOdometry();
		final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), lSpeed);
		final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), rSpeed);
		m_leftFront.set(leftOutput * 0.5);
		m_rightFront.set(rightOutput * 0.5);
		SmartDashboard.putNumber("left_out", leftOutput);
		SmartDashboard.putNumber("right_out", rightOutput);
	}

	/**
	 * PID-Controlled curvatureDrive equivalent.
	 * 
	 * @param xSpeed    Speed along the x-axis (Forward is positive)
	 * @param zRotation Rotation rate for robot (Clockwise is positive)
	 */
	public void drive(double xSpeed, double rot) {
		updateOdometry();
		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		pidTest(wheelSpeeds);
	}

	/**
	 * Shifts the pancake motors on each of the gearboxes.
	 * 
	 * @param value State to reach (Can be forward, reverse, or off.)
	 */
	public void shiftGears(Value value) {
		m_gearShift.set(value);
	}

	/**
	 * Retrieves the z-axis rotation from the PigeonIMU.
	 * 
	 * @return Current rotation around z-axis (yaw).
	 */

	public Rotation2d getAngle() {
		m_gyro.getYawPitchRoll(heading);
		return Rotation2d.fromDegrees(heading[0]);
	}

	/**
	 * Updates the odometry object with the latest sensor data.
	 */
	public void updateOdometry() {
		m_odometry.update(getAngle(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
	}

	/**
	 * Retrieves the displacement (in meters) on the left side of the drivetrain.
	 * 
	 * @return Left-side displacement
	 */
	public double getLeftDistance() {
		return m_leftEncoder.getDistance();
	}

	/**
	 * Retrieves the displacement (in meters) on the right side of the drivetrain.
	 * 
	 * @return Right-side displacement
	 */
	public double getRightDistance() {
		return m_rightEncoder.getDistance();
	}

	/**
	 * Retrieves the average displacement (in meters)between each side of the
	 * drivetrain.
	 * 
	 * @return Average displacement between each encoder.
	 */
	public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	/**
	 * Resets the encoders to zero, for autonomous commands and diagnostics.
	 */
	public void reset() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}
}
