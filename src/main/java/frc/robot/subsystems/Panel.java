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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PanelConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;

public class Panel extends SubsystemBase {
  /**
   * Creates a new Panel.
   */
  private static Panel m_instance;

  private CANSparkMax m_spinner;
  
  private CANEncoder m_spinnerEncoder; 

  private CANPIDController m_spinnerController;

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

    m_spinner.restoreFactoryDefaults();

    i2cPort = I2C.Port.kOnboard;

    colorSensor = new ColorSensorV3(i2cPort);

    m_spinnerEncoder.setPositionConversionFactor(PanelConstants.CONVERSION_FACTOR);

    //creating the object that matches the values of the Color Sensor to either Red, Green, Blue or Yellow
    matcher = new ColorMatch();

    //adding the target colors to the color matching object
    matcher.addColorMatch(blue);
    matcher.addColorMatch(green);
    matcher.addColorMatch(red);
    matcher.addColorMatch(yellow);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  
  public void spin(double speed) {
    m_spinner.set(speed);
  }

  public void voltageSpin(double input){
    m_spinnerController.setReference(input, ControlType.kVoltage);
  }
}

/*
  public Panel() {
    if (Constants.isFinal){
      m_spinner = new CANSparkMax(PortConstantsFinal.SPINNER, MotorType.kBrushless);

    }else{
      m_spinner = new CANSparkMax(PortConstants.SPINNER, MotorType.kBrushless);
    }

    m_spinnerController = m_spinner.getPIDController();

    m_spinner.restoreFactoryDefaults();
  }
*/
