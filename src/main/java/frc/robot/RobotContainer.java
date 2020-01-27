/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ShiftGears;
import frc.robot.subsystems.ComplexDrivetrain;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	// Subsystems
	public static Drivetrain drivetrain = new Drivetrain();
	public static ComplexDrivetrain test_drivetrain = new ComplexDrivetrain();
	// Commands

	// Controllers
	public static Joystick joystick = new Joystick(0);

	private RunCommand tankDrive = new RunCommand(
		() -> drivetrain.tankDrive(joystick.getRawAxis(1), joystick.getRawAxis(5)), drivetrain);
	
	private RunCommand pidTankDrive = new RunCommand(
		() -> test_drivetrain.drive(joystick.getRawAxis(1), joystick.getRawAxis(4)), test_drivetrain);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
		configureDefaultCommands();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		new JoystickButton(joystick, 4).whenPressed(new ShiftGears(drivetrain));
	}

	private void configureDefaultCommands() {
		drivetrain.setDefaultCommand(tankDrive);
		test_drivetrain.setDefaultCommand(pidTankDrive);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	/*
	 * public Command getAutonomousCommand() { return; }
	 */
}
