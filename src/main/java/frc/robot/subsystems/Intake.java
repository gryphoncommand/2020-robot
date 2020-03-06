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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  private Spark m_intake;
  private boolean limitSwitchedPressed = false;
  /*
   * Creates a new Intake.
   */
  public Intake() {
	  m_intake = new Spark(Constants.Intake.kmIntake);
  }
  /**
   * EXPERIMENTAL:
   * Runs the intake motor at a static speed.
   * Motor alignment has not yet been determined.
   */
  public void runIntake() {
  m_intake.set(0.75);
}
	public void stopIntake() {
		m_intake.set(0);
	}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
