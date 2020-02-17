/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 */
public class Colorsensing extends SubsystemBase {
    /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private Spark m_spinner;
  private double blue = 0;


  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();
  public String colorString;

  private double count = 0;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public Colorsensing() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   
    m_spinner = new Spark(0); 
  }

  // private final Joystick m_stick = new Joystick(0);
  public void spin() {
	//m_spinner.set(0.55);
	// boolean stop = m_stick.getRawButton(10); // Share

	/**
	 * The method GetColor() returns a normalized color value from the sensor and can be
	 * useful if outputting the color to an RGB LED or similar. To
	 * read the raw color, use GetRawColor().
	 * 
	 * The color sensor works best when within a few inches from an object in
	 * well lit conditions (the built in LED is a big help here!). The farther
	 * an object is the more light from the surroundings will bleed into the 
	 * measurements and make it difficult to accurately determine its color.
	 */
	Color detectedColor = m_colorSensor.getColor();

	/**
	 * Run the color match algorithm on our detected color
	 */
	ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

	if (match.color == kBlueTarget) {
	  	colorString = "Blue";
		if (blue == 0) {          
			count += 1;
    		blue = 1;
		}
	} else if (match.color == kRedTarget) {
		colorString = "Red";
		blue = 0;
	} else if (match.color == kGreenTarget) {
		colorString = "Green";
		blue = 0;
	} else if (match.color == kYellowTarget) {
		colorString = "Yellow";
		blue = 0;
	} else {
		colorString = "Unknown";
	} 
	if (count > 8) {
		m_spinner.set(0);
	} else {
		m_spinner.set(0.6);
	}

	/**
	 * Open Smart Dashboard or Shuffleboard to see the color detected by the 
	 * sensor.
	 */
	SmartDashboard.putNumber("Red", m_colorSensor.getRed());
	SmartDashboard.putNumber("Green", detectedColor.green);
	SmartDashboard.putNumber("Blue", detectedColor.blue);
	SmartDashboard.putNumber("Confidence", match.confidence);
	SmartDashboard.putString("Detected Color", colorString);
	SmartDashboard.putNumber("Count", count);
	// String detectedColor = SmartDashboard.getString("Detected Color", "none");
    m_spinner.set(0);
  }

  public void colorSpin(double colorId) {
	/**
	 * The method GetColor() returns a normalized color value from the sensor and can be
	 * useful if outputting the color to an RGB LED or similar. To
	 * read the raw color, use GetRawColor().
	 * 
	 * The color sensor works best when within a few inches from an object in
	 * well lit conditions (the built in LED is a big help here!). The farther
	 * an object is the more light from the surroundings will bleed into the 
	 * measurements and make it difficult to accurately determine its color.
	 */
	Color detectedColor = m_colorSensor.getColor();


	/**
	 * Run the color match algorithm on our detected color
	 */
	String colorString;
	ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

	if (match.color == kBlueTarget) {
		colorString = "Blue";
		if (colorId == 1) {
			count = 1;
		}
	} else if (match.color == kYellowTarget) {
		colorString = "Yellow";
		if (colorId == 2) {
			count = 1;
		}
	} else if (match.color == kRedTarget) {
		colorString = "Red";
		if (colorId == 3) {
			count = 1;
		}
	} else if (match.color == kGreenTarget) {
		colorString = "Green";
		if (colorId == 4) {
			count = 1;
		}
	} else {
		colorString = "Unknown";
	}

	/**
	 * Open Smart Dashboard or Shuffleboard to see the color detected by the 
	 * sensor.
	 */
	SmartDashboard.putNumber("Red", detectedColor.red);
	SmartDashboard.putNumber("Green", detectedColor.green);
	SmartDashboard.putNumber("Blue", detectedColor.blue);
	SmartDashboard.putNumber("Confidence", match.confidence);
	SmartDashboard.putString("Detected Color", colorString);
	if (count > 0) {
		m_spinner.set(0);
	} else {
		m_spinner.set(0.6);
	}
  }
}