package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
	private CANSparkMax m_leftFront;
	private CANSparkMax m_leftBack;
	private CANSparkMax m_rightFront;
	private CANSparkMax m_rightBack;

	private SpeedControllerGroup m_left;
	private SpeedControllerGroup m_right;

	private DifferentialDrive m_drive;

	public Drivetrain() {
		m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
		m_leftBack = new CANSparkMax(2, MotorType.kBrushless);
		m_rightFront = new CANSparkMax(3, MotorType.kBrushless);
		m_rightBack = new CANSparkMax(4, MotorType.kBrushless);

		m_left = new SpeedControllerGroup(m_leftFront, m_leftBack);
		m_right = new SpeedControllerGroup(m_rightFront, m_rightBack);

		m_drive = new DifferentialDrive(m_left, m_right);
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

}
