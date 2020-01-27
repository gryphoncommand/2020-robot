package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drivetrain extends SubsystemBase {
	private CANSparkMax m_leftFront; 
	private CANSparkMax m_leftBack;
	private CANSparkMax m_rightFront;
	private CANSparkMax m_rightBack;
	
	private DoubleSolenoid m_gearShift;	

	public Drivetrain() {
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

		m_gearShift = new DoubleSolenoid(Constants.kGearShift[0], Constants.kGearShift[1]);

		m_rightFront.setInverted(true);
		m_rightBack.setInverted(true);

	}

	public void tankDrive(double lSpeed, double rSpeed) {
		double fac = 0.25;
		lSpeed = fac * lSpeed;
		rSpeed = fac * rSpeed;
		m_leftFront.set(lSpeed);
		m_leftBack.set(lSpeed);
		m_rightFront.set(rSpeed);	
		m_rightBack.set(rSpeed);	

	}

	public void shiftGears(Value value) {
		m_gearShift.set(value);
	}
}
