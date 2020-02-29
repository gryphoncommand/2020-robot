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

public class Shooting extends SubsystemBase {
	private PunkSparkMax m_shootMotor;
	private CANEncoder m_encoder;
  /**
   * Creates a new Shooter.
   */
  public Shooting() {
	m_shootMotor = new PunkSparkMax(Constants.Shooter.kmShooter, Constants.Shooter.kConfig);
	m_encoder = m_shootMotor.getEncoder();
  }

  public void shoot(double speed) {
	m_shootMotor.set(speed);
	System.out.println(m_encoder.getVelocity());
  }
  public void stopShooting() {
	m_shootMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
