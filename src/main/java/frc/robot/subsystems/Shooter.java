/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.utils.PunkSparkMax;

import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint;
	public double prevError, error, integralpid, derivativepid, power = 0, count = 0;
	public boolean isFinished = false;

	@Log
	double setpoint = 0;

	@Log
	public double encoderCount;

	private Encoder m_encoder;
	private CANSparkMax m_shooter;

	private PIDController m_pidController;
	
	/**
	 * Creates a new Shooter.
	 */
	public Shooter() {
		m_encoder = new Encoder(6, 7, 8);
		m_shooter = new PunkSparkMax(5, Constants.Shooter.kConfig);
		
		kP = 0.03;
		kI = 0;
		kD = 0;
		kIz = 0;
		kFF = 0;
		kMaxOutput = 1;
		kMinOutput = -1;
		maxRPM = 7200;
		setPoint = 7100;

		m_pidController = new PIDController(kP, kI, kD);

		m_pidController.setIntegratorRange(kMinOutput, kMaxOutput);
		m_pidController.setTolerance(50);
		m_pidController.setP(kP);
		m_pidController.setI(kI);
		m_pidController.setD(kD);
		m_pidController.enableContinuousInput(-8000, 8000);
		m_pidController.setSetpoint(7100);
		m_pidController.setTolerance(50);
		m_pidController.setSetpoint(7100);

		dumpInfo();
	}
	
	/**
	 * Gets the current speed of the shooter.
	 * 
	 * @return Speed of shooter.
	 */
	public double getMeasurement() {
		return m_encoder.getRate();
	}

	/**
	 * Gets whether or not the shooter is at the desired speed.
	 * 
	 * @return If the shooter is at the setpoint.
	 */
	public boolean atSetpoint() {
		return m_pidController.atSetpoint();
	}

	/**
	 * Sets the shooter based on the button pressed.
	 */
	public void pid() {
		prevError = error;
		error = (m_pidController.getSetpoint() - (-1 * m_encoder.getRate()));
		integralpid = error * 0.02;
		derivativepid = (error - prevError) / 0.02;
		power += -1 * (kP * error + kI * integralpid + kD * derivativepid) / 1000;
	}

	/**
	 * Sets the shooter based on the button pressed.
	 */
	public void shoot(double powerId) {
		SmartDashboard.putNumber("Shooter - PowerID", powerId);
		
		if (powerId == 1) {
			pid();
		} else if (powerId == 2) {
			power = 0.2;
		} else if (powerId == 3) {
			power = 0.76;
		} else if (powerId == 4) {
			power = 0.80;
		} else {
			power = 0;
		}
		m_shooter.set(power);
	}

	double P, I, D = 0.002;
	double rcw, previous_error, derivative;
	/**
	 * Does PID calculation and applies them manually.
	 */
	public void manualPID() {
		
		previous_error = error;
		error = 7100 - (-1 * m_encoder.getRate()); // Error = Target - Actual
		integralpid += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
										// IterativeRobot)
		derivative = (error - previous_error) / .02;
		rcw = -1 * (P * error + I * integralpid + D * derivative) / 1000;
	}

	/**
	 * Puts constants on the SmartDashboard.
	 */
	public void dumpInfo() {
		SmartDashboard.putNumber("P Gain", kP);
		SmartDashboard.putNumber("I Gain", kI);
		SmartDashboard.putNumber("D Gain", kD);
	}

	/**
	 * Updates the PID constants from the SmartDashboard.
	 */
	public void readInfo() {
		double p = SmartDashboard.getNumber("P Gain", 0);
		double i = SmartDashboard.getNumber("I Gain", 0);
		double d = SmartDashboard.getNumber("D Gain", 0);

		// if PID coefficients on SmartDashboard have changed, write new values to
		// controller
		if ((p != kP)) {
			m_pidController.setP(p);
			kP = p;
		}
		if ((i != kI)) {
			m_pidController.setI(i);
			kI = i;
		}
		if ((d != kD)) {
			m_pidController.setD(d);
			kD = d;
		}
	}
}
