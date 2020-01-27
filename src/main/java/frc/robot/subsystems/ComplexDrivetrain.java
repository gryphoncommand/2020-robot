package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.utils.PunkSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;

public class ComplexDrivetrain extends SubsystemBase {
	private CANSparkMax m_leftFront; 
	private CANSparkMax m_leftBack;
	private CANSparkMax m_rightFront;
	private CANSparkMax m_rightBack;

	//private SpeedControllerGroup m_left;
	//private SpeedControllerGroup m_right;
	private Encoder m_leftEncoder;
	private Encoder m_rightEncoder;

	private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  	private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
	
	private DoubleSolenoid m_gearShift;	

	private DifferentialDrive m_drive;

	public ComplexDrivetrain() {
		m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
		m_leftBack = new CANSparkMax(2, MotorType.kBrushless);
		m_rightFront = new CANSparkMax(3, MotorType.kBrushless);
		m_rightBack = new CANSparkMax(4, MotorType.kBrushless);

		m_leftFront.restoreFactoryDefaults();
    	m_leftBack.restoreFactoryDefaults();
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

		m_gearShift = new DoubleSolenoid(Constants.kGearShift[0], Constants.kGearShift[1]);

		m_rightFront.setInverted(true);
		m_rightBack.setInverted(true);

		//m_left = new SpeedControllerGroup(m_leftFront, m_leftBack);
		//m_right = new SpeedControllerGroup(m_rightFront, m_rightBack);

		//m_drive = new DifferentialDrive(m_left, m_right);
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		//m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

	public void tankDrive(double lSpeed, double rSpeed) {
		// m_drive.tankDrive(lSpeed, rSpeed);
		double fac = 0.25;
		lSpeed = fac * lSpeed;
		rSpeed = fac * rSpeed;
		m_leftFront.set(lSpeed);
		m_leftBack.set(lSpeed);
		m_rightFront.set(rSpeed);	
		m_rightBack.set(rSpeed);	

	}

	public void pidTest(double lSpeed, double rSpeed) {
		final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), lSpeed);
		final double rightOutput =  m_rightPIDController.calculate(m_rightEncoder.getRate(), rSpeed);
		m_leftFront.setVoltage(leftOutput);
		m_rightFront.setVoltage(rightOutput);
	}

	public void shiftGears(Value value) {
		m_gearShift.set(value);
	}
}
