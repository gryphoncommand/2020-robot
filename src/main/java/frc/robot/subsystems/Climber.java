/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private Spark m_pvcClimber, m_wenchClimber;
  /**
   * Creates a new Climber.
   */
  public Climber() {
	m_pvcClimber = new Spark(Constants.kClimberMotor);
	m_wenchClimber = new Spark(Constants.kWenchMotor);
  }

  public void moveClimber(double speed) {
	m_pvcClimber.set(speed);
  }

  public void liftBot(double speed) {
	  m_wenchClimber.set(speed);
  }

  @Override
  public void periodic() {
	// This method will be called once per scheduler run
  }
}
