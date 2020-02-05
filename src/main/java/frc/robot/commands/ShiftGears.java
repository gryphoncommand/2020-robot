/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ComplexDrivetrain;;

public class ShiftGears extends CommandBase {
	private final ComplexDrivetrain drive;
	private boolean defaultValue;

	/**
	 * Shifts the gears of the robot. This will probably be refined down to an
	 * inline command.
	 */
	public ShiftGears(ComplexDrivetrain _drive) {
		drive = _drive;
		defaultValue = false;
		addRequirements(drive);
	}

	/**
	 * Defaults the gear-shifter.
	 */
	@Override
	public void initialize() {
		drive.shiftGears(Value.kOff);
	}

	/**
	 * Periodic function. Toggles the value of the gear-shifter.
	 */
	@Override
	public void execute() {
		if (defaultValue) {
			drive.shiftGears(Value.kForward);
		} else
			drive.shiftGears(Value.kReverse);

		defaultValue = !defaultValue;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return true;
	}
}
