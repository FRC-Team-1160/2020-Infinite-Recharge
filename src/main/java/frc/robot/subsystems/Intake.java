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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;


public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private static Intake m_instance;

  private final CANSparkMax m_indexer, m_intake, m_intakeAngle;

  double k;

  public static Intake getInstance(){
    if (m_instance == null){
      m_instance = new Intake();
    }
    return m_instance;
  }

  public Intake() {
    if (Constants.isFinal){
      m_indexer = new CANSparkMax(PortConstantsFinal.INDEXER, MotorType.kBrushed);
      m_intake = new CANSparkMax(PortConstantsFinal.INTAKE, MotorType.kBrushed);
      m_intakeAngle = new CANSparkMax(PortConstantsFinal.INTAKE_ANGLE, MotorType.kBrushless);

    }else{
      m_indexer = new CANSparkMax(PortConstants.INDEXER, MotorType.kBrushed);
      m_intake = new CANSparkMax(PortConstants.INTAKE, MotorType.kBrushed);
      m_intakeAngle = new CANSparkMax(PortConstants.INTAKE_ANGLE, MotorType.kBrushless);
    }


    m_indexer.restoreFactoryDefaults();
    m_intake.restoreFactoryDefaults();
    m_intakeAngle.restoreFactoryDefaults();

    // SmartDashboard.putNumber("input number", k);
  }

  public void intakeControl(double input){
    m_intake.setVoltage(input);
  }

  public void intakeControl(){
    m_intake.setVoltage(k);
  }

  public void indexerControl(double input){
    m_indexer.setVoltage(input);
  }

  public void intakeAngleControl(double input){
    m_intakeAngle.setVoltage(input);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // k = SmartDashboard.getNumber("input number", 0.0);
  }
}
