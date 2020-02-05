package frc.lib.utils;

import com.revrobotics.CANPIDController;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class PunkPIDController extends CANPIDController {
	public static class PidControllerConfig {
		private double P;
		private double I;
		private double D;
		private double MIN_OUTPUT;
		private double MAX_OUTPUT;
		private AlternateEncoderType ENC_TYPE;
		 
	}

	public PunkPIDController(PunkSparkMax device, PidControllerConfig config, boolean isDebug){
		super(device);
		setConfig(config);
		if(isDebug) {
			dumpData(device.name);
		}
	}
	private void dumpData(String name) {
	}
	public void setConfig(PidControllerConfig config) {
		super.setP(config.P);
		super.setI(config.I);
		super.setD(config.D);
		super.setOutputRange(config.MIN_OUTPUT, config.MAX_OUTPUT);
	}
}
