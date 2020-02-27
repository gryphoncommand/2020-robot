/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 */
public class ColorSpinner extends SubsystemBase {
	private Spark m_spinner;
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private String gameData, colorToSpin;
  private String colorString;
  private int count = 0;
  private boolean isBlue;

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

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  @Log
  private double red, blue, green, confidence;
  @Log
  private String theColor;

  public ColorSpinner() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
	m_colorMatcher.addColorMatch(kYellowTarget);  
	m_spinner = new Spark(0);
  }

  public void spinToColor() {
	spinLeft();
	if (colorToSpin == colorString) {
		m_spinner.set(0.0);
	}
  }
  public void spinFourTimes() {
	if (count < 9) {
		spinLeft();
		if ((isBlue == false) && (colorString == "Blue")) {
			count +=1;
			isBlue = true;
		}
		if (colorString != "Blue") {
			isBlue = false;
		}
	}
	m_spinner.set(0);
  }

  public void spinLeft() {
	m_spinner.set(0.6);
  }
  public void spinRight() {
	m_spinner.set(-0.6);
  }

  public void periodic() {
	gameData = DriverStation.getInstance().getGameSpecificMessage();
	if(gameData.length() > 0)
	{
	  switch (gameData.charAt(0))
	  {
		case 'B' :
		  //Blue case code
		  colorToSpin = "Blue";
		  break;
		case 'G' :
		  //Green case code
		  colorToSpin = "Green";
		  break;
		case 'R' :
		  //Red case code
		  colorToSpin = "Red";
		  break;
		case 'Y' :
		  //Yellow case code
		  colorToSpin = "Yellow";
		  break;
		default :
		  //This is corrupt data
		  System.out.println("Corrupt Data: (ColorSpinner.java:111)");
		  break;
	  }
	} else {
	  //Code for no data received yet
	}	
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
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    red = detectedColor.red;
    green = detectedColor.green;
    blue = detectedColor.blue;
    confidence = match.confidence;
    theColor = colorString;
  }
}
