package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib.utils.PunkLimelight;
import frc.robot.Constants;
import frc.robot.subsystems.ComplexDrivetrain;

public class DriveLimelight extends CommandBase {
    private final ComplexDrivetrain m_drive;
	private final NetworkTable m_limelight;
	private NetworkTableEntry m_pipeline;
	public double m_rightCommand;
	public double m_leftCommand;

    public DriveLimelight(ComplexDrivetrain drive) {
        m_drive = drive;
		m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
		m_pipeline = m_limelight.getEntry("pipeline");
		m_pipeline.setNumber(2);
		m_rightCommand = 0.0;
		m_leftCommand = 0.0;
    }

	public void execute() {
		double tx = m_limelight.getEntry("tx").getDouble(0);
		SmartDashboard.putNumber("tx", tx);

		double aimAdjust = tx/3 * Constants.Limelight.kPAngle;
		// Still needs to get target area
		m_rightCommand = aimAdjust*-1; 
		m_leftCommand =  aimAdjust*-1;

		SmartDashboard.putNumber("Right", m_leftCommand);
		SmartDashboard.putNumber("Left", m_rightCommand);

		m_drive.setLeft(m_leftCommand);
		m_drive.setRight(m_rightCommand);
    }

}
