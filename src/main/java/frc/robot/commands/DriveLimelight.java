package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utils.PunkLimelight;
import frc.robot.Constants;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.ComplexDrivetrain;

public class DriveLimelight extends CommandBase {
    private final ComplexDrivetrain m_drive;
    private final PunkLimelight m_limelight;

    public DriveLimelight(ComplexDrivetrain drive) {
        m_drive = drive;
        m_limelight = new PunkLimelight(0, null);
    }

    public double getDistance() {
        return (Constants.Limelight.kTargetHeight - Constants.Limelight.kMountHeight) / Math.tan(Constants.Limelight.kMountAngle + m_limelight.getTarget()[1]);
    }

	public void execute() {
        double tx = m_limelight.getTarget()[0];
        double ty = m_limelight.getTarget()[1];
        // implement drive to distance, angle here
    }

}
