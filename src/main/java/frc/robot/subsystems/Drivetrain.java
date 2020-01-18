package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.utils.PunkSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
	private PunkSparkMax m_leftFront; 
	private PunkSparkMax m_leftBack;
	private PunkSparkMax m_rightFront;
	private PunkSparkMax m_rightBack;

	private SpeedControllerGroup m_left;
	private SpeedControllerGroup m_right;

	private DifferentialDrive m_drive;

	public Drivetrain() {
		m_leftFront = new PunkSparkMax(
			Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig
		);
		m_leftBack = new PunkSparkMax(
			Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig, m_leftFront
		);
		m_rightFront = new PunkSparkMax(
			Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig
		);
		m_rightBack = new PunkSparkMax(
			Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig, m_rightFront
		);

		m_left = new SpeedControllerGroup(m_leftFront, m_leftBack);
		m_right = new SpeedControllerGroup(m_rightFront, m_rightBack);

		m_drive = new DifferentialDrive(m_left, m_right);
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

	public void tankDrive(double lSpeed, double rSpeed) {
		m_drive.tankDrive(lSpeed, rSpeed);
	}

}
