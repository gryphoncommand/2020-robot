package frc.lib.utils;

import edu.wpi.first.wpilibj.PWM;

public class PunkTBEncoder extends PWM {
	private float TICKS_PER_PULSE;

	public PunkTBEncoder(int channel) {
		super(channel);
		TICKS_PER_PULSE = 1000;
	}

	public float get() {
		return getRaw() * TICKS_PER_PULSE;
	} 
}
