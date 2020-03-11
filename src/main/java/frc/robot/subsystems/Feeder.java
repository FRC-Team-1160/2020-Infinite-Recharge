/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;

public class Feeder extends SubsystemBase {
  /**
   * Creates a new Delivery.
   */
  private static Feeder m_instance;

  private final CANSparkMax m_topFeeder, m_bottomFeeder;

  public static Feeder getInstance(){
    if (m_instance == null){
      m_instance = new Feeder();
    }
    return m_instance;
  }

  public Feeder() {
    if (Constants.isFinal){
      m_topFeeder = new CANSparkMax(PortConstantsFinal.TOP_FEEDER, MotorType.kBrushless);
      m_bottomFeeder = new CANSparkMax(PortConstantsFinal.BOTTOM_FEEDER, MotorType.kBrushless);

    }else{
      m_topFeeder = new CANSparkMax(PortConstants.TOP_FEEDER, MotorType.kBrushless);
      m_bottomFeeder = new CANSparkMax(PortConstants.BOTTOM_FEEDER, MotorType.kBrushless); 
    }

    m_topFeeder.restoreFactoryDefaults();
    m_bottomFeeder.restoreFactoryDefaults();

    m_topFeeder.follow(m_bottomFeeder);
  }

  /**
   * Controls the feeder of the robot.
   *
   */
  public void feederControl(double feeder){
    m_bottomFeeder.setVoltage(feeder);
    // m_bottomFeeder.setVoltage(feeder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
