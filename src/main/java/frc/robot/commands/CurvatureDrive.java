package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CurvatureDrive extends CommandBase {

	public CurvatureDrive() {
		addRequirements(RobotContainer.drivetrain);
	}

	@Override
	public void execute() {
		RobotContainer.drivetrain.curvatureDrive(RobotContainer.joystick.getRawAxis(1),
				RobotContainer.joystick.getRawAxis(2), false);
	}
}
