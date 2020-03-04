package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import frc.lib.utils.PunkLimelight;
import frc.robot.Constants;
import frc.robot.subsystems.ComplexDrivetrain;

public class DriveLimelight extends CommandBase {
    private final ComplexDrivetrain m_drive;
	private final NetworkTable m_limelight;
	private double m_rightCommand;
	private double m_leftCommand;

    public DriveLimelight(ComplexDrivetrain drive) {
        m_drive = drive;
		m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
		m_rightCommand = 0.0;
		m_leftCommand = 0.0;
    }

	public void execute() {
        double tx = m_limelight.getEntry("tx").getDouble(0);
		double ty = m_limelight.getEntry("ty").getDouble(0);
		double driveAdjust = ty * Constants.Limelight.kPDistance;
		double aimAdjust = tx * Constants.Limelight.kPAngle;
		m_rightCommand += driveAdjust - aimAdjust;
		m_leftCommand += driveAdjust + aimAdjust;

		m_drive.setLeft(m_leftCommand);
		m_drive.setRight(m_rightCommand);
    }

}
