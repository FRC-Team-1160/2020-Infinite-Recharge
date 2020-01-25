/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Panel extends SubsystemBase {
  /**
   * Creates a new Panel.
   */
  private static Panel m_instance;

  private CANSparkMax m_spinner;

  private ColorSensorV3 m_colorSensor;

  private ColorMatch m_colorMatcher;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private char curColor = 'N';

  private char gameData = 'N';

  public static Panel getInstance(){
    if (m_instance == null){
      m_instance = new Panel();
    }
    return m_instance;
  }

  public Panel() {
    m_spinner = new CANSparkMax(DriveConstants.SPINNER, MotorType.kBrushless);

    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatcher = new ColorMatch();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
  }

  public void setSpinner(double voltage){
    m_spinner.setVoltage(voltage);
  }

  public char getColor(){
    Color detectedColor = m_colorSensor.getColor();

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      curColor = 'B';
    } else if (match.color == kRedTarget) {
      curColor = 'R';
    } else if (match.color == kGreenTarget) {
      curColor = 'G';
    } else if (match.color == kYellowTarget) {
      curColor = 'Y';
    } else {
      curColor = 'U';
    }

    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", Character.toString(curColor));
    SmartDashboard.putNumber("Prox", m_colorSensor.getProximity());

    if(getProx()){
      return curColor;
    }else{
      return 'N';
    }
    
  }

  public boolean getProx(){
    if (m_colorSensor.getProximity() > 100){
      return true;
    }
    return false;
  }

  public void setTarget(){
    String tempData = DriverStation.getInstance().getGameSpecificMessage();

    if(tempData.length() > 0) {
      switch (tempData.charAt(0)){
        case 'B' :
          //Blue case code
          gameData = 'B';
          break;
        case 'G' :
          //Green case code
          gameData = 'B';
          break;
        case 'R' :
          //Red case code
          gameData = 'R';
          break;
        case 'Y' :
          //Yellow case code
          gameData = 'Y';
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      gameData = 'N';
      //Code for no data received yet
    }
  }

  public char getTarget(){
    return gameData;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getColor();    
    setTarget();
  }
}
