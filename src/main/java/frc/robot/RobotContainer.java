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
import frc.robot.subsystems.ColorSpinner;
import frc.robot.subsystems.ComplexDrivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooting;
// import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveToDistance;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems
	public static ComplexDrivetrain drivetrain = new ComplexDrivetrain();
	// public static ColorSpinner colorsensor = new ColorSpinner();
	public static Shooting shooter = new Shooting();
	public static Intake intake = new Intake();
	public static Index index = new Index();
	public static Climber climber = new Climber();

	// Controllers
	public static Joystick joystick = new Joystick(0);
	public static JoystickButton square = new JoystickButton(joystick, 1);
	public static JoystickButton circle = new JoystickButton(joystick, 3);
	public static JoystickButton triangle = new JoystickButton(joystick, 4);
	public static JoystickButton leftTrigger = new JoystickButton(joystick, 7);
	public static JoystickButton rightTrigger = new JoystickButton(joystick, 8);
	public static JoystickButton xbutton = new JoystickButton(joystick, 2);
	// Command
	// private RunCommand pidTankDrive = new RunCommand(
	// 		() -> drivetrain.pidTankDrive(joystick.getRawAxis(1), joystick.getRawAxis(5)), drivetrain);
	private RunCommand pidTankDrive = new RunCommand(
			() -> drivetrain.tankDrive(joystick.getRawAxis(1), joystick.getRawAxis(5)), drivetrain);
	// private RunCommand testOnboardPID = new RunCommand(
	// 	() -> drivetrain.setVelocity(joystick.getRawAxis(1), joystick.getRawAxis(5)), drivetrain);
	private RunCommand testOnboardPID = new RunCommand(
		() -> drivetrain.setVelocity(-1, joystick.getRawAxis(5)), drivetrain);
	private RunCommand runIntake = new RunCommand(()-> {
		if(triangle.get()) {
			intake.runIntake();
		} else {
			intake.stopIntake();
		}
	}, intake);
	private RunCommand runIndexer = new RunCommand(()-> {
		if(circle.get()) {
			index.reverse();
		} else {
			index.stopIndexer();
		}
		if(rightTrigger.get()) {
			index.runIndexer();
		} else {
			index.stopIndexer();
		}
	}, index);

	private RunCommand runShooter = new RunCommand(()-> {
		if(leftTrigger.get()) {
			shooter.shoot(-0.53);
		} else if (xbutton.get()) {
			shooter.shoot(0.25);
		} else {
			shooter.stopShooting();
		}
	}, shooter);
	
	private POVButton telUp = new POVButton(joystick, 0);
	private POVButton telRight = new POVButton(joystick, 90);
	private POVButton telDown = new POVButton(joystick, 180);
	private POVButton telLeft = new POVButton(joystick, 270);
	private RunCommand runClimber = new RunCommand(() -> {
		if(telLeft.get()) {
			climber.moveClimber(-1);
		} else if(telRight.get()) {
			climber.moveClimber(1);
		} else if(telUp.get()) {
			climber.liftBot(-1);
		} else if(telDown.get()) {
			climber.liftBot(1);
		} else {
			climber.moveClimber(0);
			climber.liftBot(0);
		}
	}, climber);
	// private RunCommand colorSensor = new RunCommand(
	// 		() -> colorsensor.periodic(), colorsensor);

	// private RunCommand curvatureDrive = new RunCommand(
			// () -> drivetrain.curvatureDrive(joystick.getRawAxis(1), joystick.getRawAxis(2)), drivetrain);\
	// FOR TESTING PURPOSES ONLY - Runs the shooter based off of the SmartDashboard
	//private RunCommand testShooter = new RunCommand(() -> shooter.shooterPeriodic(), shooter);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */

	public RobotContainer() {
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
		// Triangle - Intake
		// new JoystickButton(joystick, 4).whenPressed(new InstantCommand(() -> intake.runIntake(), intake));
		// X - Shift Gears
		new JoystickButton(joystick, 2).whenPressed(new ShiftGears(drivetrain));
		// Right Trigger - Index
		// new JoystickButton(joystick, 8).whenPressed(new InstantCommand(() -> intake.runIndexer(), intake));
		// Left Trigger - Shooter
		// new JoystickButton(joystick, 7).whenPressed(new InstantCommand(() -> shooter.shoot(), shooter));
		// Circle - Index Reverse
		// new JoystickButton(joystick, 3).whenPressed(new InstantCommand(() -> intake.reverse(), shooter));
		/**
		// Share - Color Sensor
		new JoystickButton(joystick, 9).whenPressed(new InstantCommand(
			() -> colorsensor.spinToColor(), colorsensor));
			// Share - Color Sensor
		new JoystickButton(joystick, 9).whenPressed(new InstantCommand(
			() -> colorsensor.spinFourTimes(), colorsensor));
		// Left Bumper - Spin Wheel Left
		new JoystickButton(joystick, 5).whenPressed(new InstantCommand(
			() -> colorsensor.spinLeft(), colorsensor));
		// Right Bumper - Spin Wheel Right
		new JoystickButton(joystick, 6).whenPressed(new InstantCommand(
			() -> colorsensor.spinRight(), colorsensor));
		 */
		/**
		 * POV Functionality
		 * For reference:
		 * 0 - Up
		 * 90 - Right
		 * 180 - Down
		 * 270 - Left
		 */ 
		// new POVButton(joystick, 0).whenPressed(new InstantCommand(()->climber.moveClimber(0.5), climber));
		// new POVButton(joystick, 180).whenPressed(new InstantCommand(()->climber.moveClimber(-0.5), climber));
		// new POVButton(joystick, 0).whenPressed(new InstantCommand(()->climber.liftBot(1), climber));
		// new POVButton(joystick, 180).whenPressed(new InstantCommand(()->climber.liftBot(-1), climber));

	}

	private void configureDefaultCommands() {
		drivetrain.setDefaultCommand(pidTankDrive);
		// colorsensor.setDefaultCommand(colorSensor);
		shooter.setDefaultCommand(runShooter);
		intake.setDefaultCommand(runIntake);
		index.setDefaultCommand(runIndexer);
		climber.setDefaultCommand(runClimber);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */

	public Command getAutonomousCommand() { 
		return new DriveToDistance(-2, drivetrain);
	}
	
}
