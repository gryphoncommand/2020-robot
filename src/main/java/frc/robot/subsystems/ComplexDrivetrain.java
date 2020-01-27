package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.sensors.PigeonIMU;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;


public class ComplexDrivetrain extends SubsystemBase {
	private CANSparkMax m_leftFront; 
	private CANSparkMax m_leftBack;
	private CANSparkMax m_rightFront;
	private CANSparkMax m_rightBack;

	private Encoder m_leftEncoder;
	private Encoder m_rightEncoder;
	private AHRS m_navx;

	private PigeonIMU m_gyro;

	private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  	private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
	
	private DoubleSolenoid m_gearShift;	

	private final DifferentialDriveKinematics m_kinematics = 
		new DifferentialDriveKinematics(0.645668); // Fill with track width (in meters)
	private final DifferentialDriveOdometry m_odometry;

	public ComplexDrivetrain() {
		m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
		m_leftBack = new CANSparkMax(2, MotorType.kBrushless);
		m_rightFront = new CANSparkMax(3, MotorType.kBrushless);
		m_rightBack = new CANSparkMax(4, MotorType.kBrushless);

		m_leftFront.restoreFactoryDefaults();
    	m_leftBack.restoreFactoryDefaults();
    	m_rightFront.restoreFactoryDefaults();
    	m_rightBack.restoreFactoryDefaults();

		m_leftFront.setIdleMode(IdleMode.kCoast);
		m_leftBack.setIdleMode(IdleMode.kCoast);
		m_rightFront.setIdleMode(IdleMode.kCoast);
		m_rightBack.setIdleMode(IdleMode.kCoast);

		m_leftEncoder = new Encoder(0, 1, 2);
		m_rightEncoder = new Encoder(3, 4, 5);

		m_leftEncoder.setDistancePerPulse(2 * Math.PI * 0.1524 / 8192);
		m_rightEncoder.setDistancePerPulse(2 * Math.PI * 0.1524 / 8192);
		m_leftEncoder.reset();
		m_rightEncoder.reset();	
		m_navx = new AHRS(SPI.Port.kMXP);

		m_gyro = new PigeonIMU(0);
		m_gearShift = new DoubleSolenoid(Constants.kGearShift[0], Constants.kGearShift[1]);

		m_rightFront.setInverted(true);
		m_rightBack.setInverted(true);

		m_odometry = new DifferentialDriveOdometry(getAngle());
	}

	// Need to calibrate and see Pigeon orientation.
	public Rotation2d getAngle() {
		double head = (double) m_navx.getYaw();
		return Rotation2d.fromDegrees(head);
	}
	//Lexi was here and theres nothing you can do about it :)
	/**
	 *  "It's OK, Lexi." - Tyler Duckworth
	 */	
	public void tankDrive(double lSpeed, double rSpeed) {
		double fac = 0.25;
		lSpeed = fac * lSpeed;
		rSpeed = fac * rSpeed;
		m_leftFront.set(lSpeed);
		m_leftBack.set(lSpeed);
		m_rightFront.set(rSpeed);	
		m_rightBack.set(rSpeed);	

	}

	// public void pidTest(DifferentialDriveWheelSpeeds _speeds) {
	// 	final double leftOutput = m_leftPIDController.calculate(
	// 		m_leftEncoder.getRate(), _speeds.leftMetersPerSecond);
	// 	final double rightOutput =  m_rightPIDController.calculate(
	// 		m_rightEncoder.getRate(), _speeds.rightMetersPerSecond);
	// 	m_leftFront.setVoltage(leftOutput);
	// 	m_rightFront.setVoltage(rightOutput);
	// }
	public void pidTest(double lSpeed, double rSpeed) {
		final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), lSpeed);
		final double rightOutput =  m_rightPIDController.calculate(m_rightEncoder.getRate(), rSpeed);
		m_leftFront.setVoltage(leftOutput);
		m_rightFront.setVoltage(rightOutput);
	}

	public void drive(double xSpeed, double rot) {
		updateOdometry();
		var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		//pidTest(wheelSpeeds);
	}

	public void shiftGears(Value value) {
		m_gearShift.set(value);
	}
	public void updateOdometry() {
		m_odometry.update(getAngle(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
	}
}
