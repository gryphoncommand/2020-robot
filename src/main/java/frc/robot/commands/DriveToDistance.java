/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.ComplexDrivetrain;

public class DriveToDistance extends PIDCommand {
	private final ComplexDrivetrain m_drivetrain;

	/**
	 * Drives a certain distance.
	 * 
	 * @param distance    Distance (in meters) that you want to drive
	 * @param _drivetrain Injected drivetrain object.
	 */
	public DriveToDistance(double distance, ComplexDrivetrain _drivetrain) {
		super(
				// The controller that the command will use
				new PIDController(4, 0, 0),
				// This should return the measurement
				_drivetrain::getDistance,
				// This should return the setpoint (can also be a constant)
				distance,
				// This uses the output
				setPoint -> {
					_drivetrain.pidTankDrive(setPoint, setPoint);
				});

		m_drivetrain = _drivetrain;
		addRequirements(m_drivetrain);
		getController().setTolerance(0.01);
	}

	/**
	 * Defaults the encoders to zero.
	 */
	@Override
	public void initialize() {
		// Get everything in a safe starting state.
		m_drivetrain.reset();
		super.initialize();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return getController().atSetpoint();
	}
}
