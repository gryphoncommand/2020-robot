/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;;

public class ShiftGears extends CommandBase {
  private final Drivetrain drive;
  private boolean defaultValue;
  public ShiftGears(Drivetrain _drive) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    drive = _drive;
    defaultValue = false;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.shiftGears(Value.kOff);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(defaultValue) {
        drive.shiftGears(Value.kForward);
    } else drive.shiftGears(Value.kReverse);

    defaultValue = !defaultValue;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }
}