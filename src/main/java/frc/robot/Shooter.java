/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.PWMVictorSPX;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends SubsystemBase {
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint;

  private Encoder m_encoder;
  private CANSparkMax m_shooter;

  private PIDController m_pidController;
  public double power = 0;
  public double count = 0;
  public double encoderCount;
  public boolean isFinished = false;  

  double setpoint = 0;


  public Shooter() {
    m_encoder = new Encoder(6,7,8);
    kP = 0.03; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 7200;
    setPoint = 7100;
    SmartDashboard.putNumber("SetPoint", setPoint);
    m_shooter = new CANSparkMax(5, MotorType.kBrushless);
    m_pidController = new PIDController(kP,kI,kD);
    m_pidController.setIntegratorRange(kMinOutput, kMaxOutput);
    m_pidController.setTolerance(50);
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.enableContinuousInput(-8000, 8000);
    m_pidController.setSetpoint(7100);
    updateConstants();
    dumpInfo();
  }

  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------

  // public void shoot(double powerId) {
  //   encoderCount = -1*m_encoder.getRate();
  //   if (powerId == 1) {
  //     double error = Math.abs(setPoint-encoderCount);
  //     boolean errorZone = (error < 50);
  //     if (errorZone) {
  //       count += 1;
  //       isFinished = count >= 5;
  //     } else {
  //       count = 0;
  //     }
  //     if (!isFinished) {
  //       readInfo();
  //       double pidOut = m_pidController.calculate((-1*encoderCount), 7150);
        
  //       SmartDashboard.putNumber("ProcessVariable", encoderCount);
  //       power += (error/9300);
  //       //power += pidOut;
  //       m_shooter.set(power);
  //       System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
  //     }
  //   } else if (powerId == 2){
  //     m_shooter.set(-0.72);
  //     System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
  //   } else if (powerId == 3) {
  //     m_shooter.set(-0.76);
  //     System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
  //   } else if (powerId == 4) {
  //     m_shooter.set(-0.80);
  //     System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
  //   } else {
  //     m_shooter.set(0);
  //   }
  // }

  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------

  /**
   * The shooter subsystem for the robot.
   */
  public void ShooterSubsystem() {
    m_pidController.setTolerance(50);
    m_pidController.setSetpoint(7100);
  }
  public double getMeasurement() {
    return m_encoder.getRate();
  }
  public boolean atSetpoint() {
    return m_pidController.atSetpoint();
  }
  double prevError, error, integralpid, derivativepid;
  public void pid() {
    prevError = error;
    error = (m_pidController.getSetpoint()-(-1*m_encoder.getRate()));
    integralpid = error*0.02;
    derivativepid = (error - prevError)/0.02;
    power += -1*(kP*error + kI*integralpid + kD*derivativepid)/1000;
  }

  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------

  public void shoot(double powerId) {
    if (powerId == 1) {
      pid();
      m_shooter.set(power);
      System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
    } else if (powerId == 2){
        m_shooter.set(0.2);
        System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
    } else if (powerId == 3) {
        m_shooter.set(0.76);
        System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
    } else if (powerId == 4) {
        m_shooter.set(0.80);
        System.out.println("Encoder Count: " + encoderCount + ", PowerID: " + powerId);
    } else {
        m_shooter.set(0);
    }
  }

  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------

  //
  double P, I, D = 0.002;
  private double rcw, previous_error, derivative;
  public void pID() {
    previous_error = error;
    error = 7100 - (-1*m_encoder.getRate()); // Error = Target - Actual
    integralpid += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - previous_error) / .02;
    rcw = -1*(P*error + I*integralpid + D*derivative)/1000;
  }

  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------

  // public void shoot(double powerId) {
  //   pID();
  //   m_shooter.set(rcw);
  // }

  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
  
  public void stop() {
    //controller.setReference(0, ControlType.kDutyCycle);
  }
  public void dumpInfo() {
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
  }
  public void readInfo() {
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
  }

  // public void update() {
  //   FireLog.log("shooterVelocity", Math.abs(encoder.getVelocity()));
  // }

  public void updateConstants() {
    // controller.setOutputRange(-1, 0);
    // controller.setP(0.0011);
    // controller.setI(0);
    // controller.setD(4);
    // controller.setFF(0.00017);
  }
}