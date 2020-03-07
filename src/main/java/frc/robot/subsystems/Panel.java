/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.subsystems;
 
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PanelConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;
 
public class Panel extends SubsystemBase {
  //Creates a new Panel.
  private static Panel m_instance;
  private final I2C.Port m_i2cPort;
 
  private final ColorSensorV3 m_colorSensor;
  private ColorMatch m_colorMatcher;
 
  private CANSparkMax m_spinner;
  
  private CANEncoder m_spinnerEncoder; 
 
  private CANPIDController m_spinnerController;
 
  //setting the target colors (calibrated)
  private final Color blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color green = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color red = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  //information on the color that we need to find in Position Control
  private Color m_targetColor;
  private String m_gameData;
 
 
  public static Panel getInstance() {
    if (m_instance == null) {
      m_instance = new Panel();
    }
    return m_instance;
  }
 
  public Panel() {
    if (Constants.isFinal){
      m_spinner = new CANSparkMax(PortConstantsFinal.SPINNER, MotorType.kBrushless);
 
    }else{
      m_spinner = new CANSparkMax(PortConstants.SPINNER, MotorType.kBrushless);
    }
 
    m_spinnerEncoder = m_spinner.getEncoder();
 
    m_spinnerController = m_spinner.getPIDController();
 
    //resetting the motor to avoid unwanted errors
    m_spinner.restoreFactoryDefaults();
 
    m_spinner.restoreFactoryDefaults();
 
    m_i2cPort = I2C.Port.kOnboard;
 
    m_colorSensor = new ColorSensorV3(m_i2cPort);
 
    // m_spinnerEncoder.setPositionConversionFactor(PanelConstants.CONVERSION_FACTOR);
 
    //creating the object that matches the values of the Color Sensor to either Red, Green, Blue or Yellow
    m_colorMatcher = new ColorMatch();
 
    //adding the target colors to the color matching object
    m_colorMatcher.addColorMatch(blue);
    m_colorMatcher.addColorMatch(green);
    m_colorMatcher.addColorMatch(red);
    m_colorMatcher.addColorMatch(yellow);   
  }
  
  public void spin(double speed) {
    m_spinner.setVoltage(speed);
  }
 
  public void voltageSpin(double input) {
    m_spinnerController.setReference(input, ControlType.kVoltage);
  }
 
  public boolean foundColor(){
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult detectedMatch = m_colorMatcher.matchClosestColor(detectedColor);
    ColorMatchResult targetMatch = m_colorMatcher.matchClosestColor(m_targetColor);
    if(detectedMatch.color == targetMatch.color){
      return true;
    } else {
      return false;
    }
  }
 
  public CANEncoder getPanelEncoder(){
    return m_spinnerEncoder;
  }
 
  /*
  public void setTargetColor(){
    switch (m_gameData.charAt(0)){
      case 'B' :
        //To land on blue, red has to be under the color sensor
        m_targetColor = red;
        break;
      case 'G' :
        //To land on green, yellow has to be under the color sensor
        m_targetColor = yellow;
        break;
      case 'R' :
        //To land on red, blue has to be under the color sensor 
        m_targetColor = blue;
        break;
      case 'Y' :
        //To land on yellow, green has to be under the color sensor
        m_targetColor = green;
        break;
      default :
        //This is corrupt data
        System.out.println("No Color");
        break;
    }
  }
 */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_gameData = DriverStation.getInstance().getGameSpecificMessage();
    SmartDashboard.putString("Game Data (Target Color)", m_gameData);
 //   setTargetColor();
 
 
    SmartDashboard.putNumber("Encoder Position", m_spinnerEncoder.getPosition());
    SmartDashboard.putNumber("Encoder Counts per Revolution", m_spinnerEncoder.getCountsPerRevolution());
  }
}
