/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PanelConstants;

public class Panel extends SubsystemBase {
  //Creates a new Panel.
  private static Panel m_instance;

  private CANSparkMax panelMotor;
  private CANEncoder panelEncoder; 
  private final I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;
  private ColorMatch matcher;

  //setting the target colors (calibrated)
  private final Color blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color green = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color red = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  //information on the color that we need to find in Position Control
  private Color targetColor;
  private String gameData;

  public static Panel getInstance(){
    if (m_instance == null){
      m_instance = new Panel();
    }
    return m_instance;
  }

  public Panel() {
    //May need to change the I2C port below to match the connection of the color sensor
    i2cPort = I2C.Port.kOnboard;

    colorSensor = new ColorSensorV3(i2cPort);
    panelMotor = new CANSparkMax(PanelConstants.PANEL, MotorType.kBrushless);
    panelEncoder = panelMotor.getEncoder(EncoderType.kHallSensor, PanelConstants.COUNTS_PER_REV);

    //resetting the motor to avoid unwanted errors
    panelMotor.restoreFactoryDefaults();

    //Set the conversion factor for position of the encoder. 
    //This factor is multiplied by the native output units of the panelEncoder to give the position.
    panelEncoder.setPositionConversionFactor(PanelConstants.CONVERSION_FACTOR);

    //creating the object that matches the values of the Color Sensor to either Red, Green, Blue or Yellow
    matcher = new ColorMatch();

    //adding the target colors to the color matching object
    matcher.addColorMatch(blue);
    matcher.addColorMatch(green);
    matcher.addColorMatch(red);
    matcher.addColorMatch(yellow);   
  }

  //getter methods so that the Commands can reach the private variables
  public CANSparkMax getPanelMotor(){
    return panelMotor;
  }

  public CANEncoder getPanelEncoder(){
    return panelEncoder;
  }

  public ColorSensorV3 getColorSensor(){
    return colorSensor;
  }

  public ColorMatch getMatcher(){
    return matcher;
  }

  public Color getTargetColor(){
    return targetColor;
  }

  public void setTargetColor(){
    switch (gameData.charAt(0)){
      case 'B' :
        //To land on blue, red has to be under the color sensor
        targetColor = red;
        break;
      case 'G' :
        //To land on green, yellow has to be under the color sensor
        targetColor = yellow;
        break;
      case 'R' :
        //To land on red, blue has to be under the color sensor 
        targetColor = blue;
        break;
      case 'Y' :
        //To land on yellow, green has to be under the color sensor
        targetColor = green;
        break;
      default :
        //This is corrupt data
        System.out.println("No Color");
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    //display color on smartdashboard
    SmartDashboard.putString("Game Data (Target Color)", gameData);

    if(!gameData.equals("")){
      setTargetColor();
    }
  }
}
