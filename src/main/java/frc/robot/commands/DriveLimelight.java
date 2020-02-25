package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utils.PunkLimelight;
import frc.robot.Constants;
import frc.robot.subsystems.ComplexDrivetrain;

public class DriveLimelight extends CommandBase {
    private final ComplexDrivetrain m_drive;
	private final PunkLimelight m_limelight;
	private double m_rightCommand;
	private double m_leftCommand;

    public DriveLimelight(ComplexDrivetrain drive) {
        m_drive = drive;
		m_limelight = new PunkLimelight(0, Constants.kLimelight);
		m_rightCommand = 0.0;
		m_leftCommand = 0.0;
    }

    public double getDistance() {
        return (Constants.Limelight.kTargetHeight - Constants.Limelight.kMountHeight) / Math.tan(Constants.Limelight.kMountAngle + m_limelight.getTarget()[1]);
    }

	public void execute() {
        double tx = m_limelight.getTarget()[0];
		double ty = m_limelight.getTarget()[1];
		double driveAdjust = ty * Constants.Limelight.kPDistance;
		double aimAdjust = tx * Constants.Limelight.kPAngle;
		m_rightCommand += driveAdjust - aimAdjust;
		m_leftCommand += driveAdjust + aimAdjust;

		m_drive.setLeft(m_rightCommand);
		m_drive.setRight(m_leftCommand);
    }

}
