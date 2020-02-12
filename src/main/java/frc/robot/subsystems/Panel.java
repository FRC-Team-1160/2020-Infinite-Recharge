/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import static edu.wpi.first.wpilibj.templates.commandbased.Constants.OIConstants.*;
import frc.robot.Constants.*;

public class Panel extends SubsystemBase {
  /**
   * Creates a new Panel.
   */
  private final I2C.Port i2cPort;
  private final ColorSensorV3 m_colorSensor;
  private CANSparkMax panelMotor; 
  private int colorCounter;
  private Color currentColor;
  private Color previousColor;
  private ColorMatch matcher;

  //setting the target colors (calibrated)
  private final Color blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color green = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color red = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private Color targetColor;

  public Panel() {
    //Change the I2C port below to match the connection of the color sensor
    i2cPort = I2C.Port.kOnboard;

    m_colorSensor = new ColorSensorV3(i2cPort);
    panelMotor = new CANSparkMax(PanelConstants.PANEL, MotorType.kBrushless);

    //create the counter that will see when the color changes while the panel spins
    colorCounter = 0;

    //the colors that the variables are set to right now are just placeholders (to mitigate future errors when doing comparison tests)
    currentColor = m_colorSensor.getColor(); 
    previousColor = m_colorSensor.getColor();

    //creating the object that matches the values of the Color Sensor to either Red, Green, Blue or Yellow
    matcher = new ColorMatch();

    //adding the target colors to the color matching object
    matcher.addColorMatch(blue);
    matcher.addColorMatch(green);
    matcher.addColorMatch(red);
    matcher.addColorMatch(yellow);   

  }

  public void startSpinning(double voltage){
    //the motor is reset and then it spins
    panelMotor.restoreFactoryDefaults();
    panelMotor.set(voltage);
  }

  public void stopSpinning(){
    //stop the motor & reset it
    panelMotor.stopMotor();
    panelMotor.restoreFactoryDefaults();
  }

  public boolean enoughSpins(){
    //updates the previousColor and currentColor variables
    previousColor = currentColor;
    currentColor = m_colorSensor.getColor();
    //seeing if the previous color is NOT the same as the current color detected (looking for a change in color)
    if(!matcher.matchClosestColor(previousColor).equals(matcher.matchClosestColor(currentColor))){
      colorCounter++;
      if(colorCounter > 26) {
        return true;
      }
    }
    return false;
  }

  public void setTargetColor(){
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    switch (gameData.charAt(0)){
      case 'B' :
        //Blue case code
        targetColor = blue;
        break;
      case 'G' :
        //Green case code
        targetColor = green;
        break;
      case 'R' :
        //Red case code
        targetColor = red;
        break;
      case 'Y' :
        //Yellow case code
        targetColor = yellow;
        break;
      default :
        //This is corrupt data
        break;
    }
  }

  public boolean foundColor(){
    currentColor = m_colorSensor.getColor();
    //to see if the current Color detected by the wheel is the color we want
    if((matcher.matchClosestColor(targetColor).equals(matcher.matchClosestColor(currentColor))){
      return true;
    }
    return false;
  }

  //put the methods that do actions

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
