package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.utils.PunkSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drivetrain extends SubsystemBase {
	private CANSparkMax m_leftFront; 
	private CANSparkMax m_leftBack;
	private CANSparkMax m_rightFront;
	private CANSparkMax m_rightBack;

	private SpeedControllerGroup m_left;
	private SpeedControllerGroup m_right;
	
	private DoubleSolenoid m_gearShift;	

	private DifferentialDrive m_drive;

	public Drivetrain() {
		// m_leftFront = new PunkSparkMax(
		// 	Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig
		// );
		// m_leftBack = new PunkSparkMax(
		// 	Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig, m_leftFront
		// );
		// m_rightFront = new PunkSparkMax(
		// 	Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig
		// );
		// m_rightBack = new PunkSparkMax(
		// 	Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig, m_rightFront
		// );
		m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
		m_leftBack = new CANSparkMax(2, MotorType.kBrushless);
		m_rightFront = new CANSparkMax(3, MotorType.kBrushless);
		m_rightBack = new CANSparkMax(4, MotorType.kBrushless);

		m_gearShift = new DoubleSolenoid(Constants.kGearShift[0], Constants.kGearShift[1]);

		m_rightFront.setInverted(true);
		m_rightBack.setInverted(true);

		m_left = new SpeedControllerGroup(m_leftFront, m_leftBack);
		m_right = new SpeedControllerGroup(m_rightFront, m_rightBack);

		m_drive = new DifferentialDrive(m_left, m_right);
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		m_drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

	public void tankDrive(double lSpeed, double rSpeed) {
		// m_drive.tankDrive(lSpeed, rSpeed);
		m_leftFront.set(lSpeed);
		m_leftBack.set(lSpeed);
		m_rightFront.set(rSpeed);
		m_rightBack.set(rSpeed);
	}

	public void shiftGears(Value value) {
		m_gearShift.set(value);
	}
}
