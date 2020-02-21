/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Feeder extends SubsystemBase {
  /**
   * Creates a new Delivery.
   */
  private static Feeder m_instance;

  private final CANSparkMax m_topFeeder, m_bottomFeeder;

  private final DifferentialDrive m_mainFeeder;

  public static Feeder getInstance(){
    if (m_instance == null){
      m_instance = new Feeder();
    }
    return m_instance;
  }

  public Feeder() {
    m_topFeeder = new CANSparkMax(PortConstants.TOP_FEEDER, MotorType.kBrushless);
    m_bottomFeeder = new CANSparkMax(PortConstants.BOTTOM_FEEDER, MotorType.kBrushless);

    m_topFeeder.restoreFactoryDefaults();
    m_bottomFeeder.restoreFactoryDefaults();
    
    m_mainFeeder = new DifferentialDrive(m_topFeeder, m_bottomFeeder);
  }

  public void feederControl(double feeder){
    m_mainFeeder.tankDrive(feeder, -feeder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
