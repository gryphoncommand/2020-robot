/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.PunkSparkMax;
import frc.robot.Constants;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Shooting extends SubsystemBase {
	private PunkSparkMax m_shootMotor;
	private CANEncoder m_encoder;
	private double newSpeed = -0.75;
	private final NetworkTable m_limelight;
	private NetworkTableEntry m_pipeline;
  /**
   * Creates a new Shooter.
   */
  public Shooting() {
	m_shootMotor = new PunkSparkMax(Constants.Shooter.kmShooter, Constants.Shooter.kConfig);
	m_encoder = m_shootMotor.getEncoder();
	m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
  	m_pipeline = m_limelight.getEntry("pipeline");
  	m_pipeline.setNumber(2);
  }

  public void shootLimeLight() {
	double ty = m_limelight.getEntry("ty").getDouble(0);
	if (ty > 10) {
		newSpeed = -0.75;
	} else if (ty > 5) {
		newSpeed = -0.68;
	} else if (ty > 1) {
		newSpeed = -0.60;
	} else {
		newSpeed = -0.56;
	}

	m_shootMotor.set(newSpeed);
	System.out.println(m_encoder.getVelocity());
  }

  public void shoot(double speed) {
	m_shootMotor.set(speed);
	System.out.println(m_encoder.getVelocity());
  }

  public void stopShooting() {
	m_shootMotor.set(0.0);
  }

}
