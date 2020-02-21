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
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Panel extends SubsystemBase {
  /**
   * Creates a new Panel.
   */
  private static Panel m_instance;

  private CANSparkMax m_spinner;

  private CANPIDController m_spinnerController;

  public static Panel getInstance() {
    if (m_instance == null) {
      m_instance = new Panel();
    }
    return m_instance;
  }

  public Panel() {
    m_spinner = new CANSparkMax(PortConstants.SPINNER, MotorType.kBrushless);

    m_spinnerController = m_spinner.getPIDController();

    m_spinner.restoreFactoryDefaults();
  }

  public void spin(double speed) {
    m_spinner.set(speed);
  }

  public void voltageSpin(double input){
    m_spinnerController.setReference(input, ControlType.kVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
