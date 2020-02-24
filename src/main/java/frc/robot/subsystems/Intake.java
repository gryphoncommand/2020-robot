/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.lib.utils.PunkLimelight.LimelightConfig;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.subsystems.Shooter;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

public class Intake extends SubsystemBase {
	public boolean isFinished = false;
	private Shooter m_shooting;

	private Spark m_intakeMotor;
	public DigitalInput m_LimitSwitch;
	
	/**
	 * Creates a new Shooter.
	 */
	public Intake() {
		m_intakeMotor = new Spark(0);
		m_shooting = new Shooter();
		m_intakeMotor.set(0.0);
		m_shooting.m_hopperMotor.set(0.0);
		m_LimitSwitch = new DigitalInput(1);
	}

	/**
	 * Sets the shooter based on the button pressed.
	 */
	public void pullIntake() {
		m_intakeMotor.set(0.5);
		m_shooting.m_hopperMotor.set(0.5);
		if (m_LimitSwitch.get()) {
			m_shooting.m_hopperMotor.set(0.0);
		}
	}

	/**
	 * Puts constants on the SmartDashboard.
	 */
	public void dumpInfo() {
		
	}
}
