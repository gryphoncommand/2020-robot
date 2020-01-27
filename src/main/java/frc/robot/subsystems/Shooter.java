/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.kauailabs.navx.frc.AHRS;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotor;
  private CANEncoder m_encoder;
  private CANPIDController m_controller;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
	m_shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
	m_shooterMotor.restoreFactoryDefaults();

	m_encoder = m_shooterMotor.getEncoder();
	m_controller = m_shooterMotor.getPIDController();
	kP = 5e-5; 
	kI = 1e-6;
	kD = 0; 
	kIz = 0; 
	kFF = 0; 
	kMaxOutput = 1; 
	kMinOutput = -1;
	maxRPM = 5700;

	m_controller.setP(kP);
	m_controller.setI(kI);
	m_controller.setD(kD);
	m_controller.setIZone(kIz);
	m_controller.setFF(kFF);
	m_controller.setOutputRange(kMinOutput, kMaxOutput);
	dumpInfo();
  }
  private void dumpInfo() {
	SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }
  private void getInfo() {
	double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_controller.setP(p); kP = p; }
    if((i != kI)) { m_controller.setI(i); kI = i; }
    if((d != kD)) { m_controller.setD(d); kD = d; }
    if((iz != kIz)) { m_controller.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_controller.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
	  m_controller.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
