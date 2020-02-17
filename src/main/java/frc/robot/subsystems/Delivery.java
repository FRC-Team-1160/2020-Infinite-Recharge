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
import frc.robot.Constants.PortConstants;

public class Delivery extends SubsystemBase {
  /**
   * Creates a new Delivery.
   */
  private static Delivery m_instance;

  private final CANSparkMax m_leftShooter, m_rightShooter,
    m_topFeeder, m_bottomFeeder,
    m_indexer, m_intake
  ;

  private final DifferentialDrive m_mainShoot, m_mainFeeder, m_mainIntake;

  public static Delivery getInstance(){
    if (m_instance == null){
      m_instance = new Delivery();
    }
    return m_instance;
  }

  public Delivery() {
    m_leftShooter = new CANSparkMax(PortConstants.LEFT_SHOOTER, MotorType.kBrushless);
    m_rightShooter = new CANSparkMax(PortConstants.RIGHT_SHOOTER, MotorType.kBrushless);
  
    m_topFeeder = new CANSparkMax(PortConstants.TOP_FEEDER, MotorType.kBrushless);
    m_bottomFeeder = new CANSparkMax(PortConstants.BOTTOM_FEEDER, MotorType.kBrushless);

    m_indexer = new CANSparkMax(PortConstants.INDEXER, MotorType.kBrushless);
    m_intake = new CANSparkMax(PortConstants.INTAKE, MotorType.kBrushless);

    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();

    m_topFeeder.restoreFactoryDefaults();
    m_bottomFeeder.restoreFactoryDefaults();

    m_indexer.restoreFactoryDefaults();
    m_intake.restoreFactoryDefaults();

    m_mainShoot = new DifferentialDrive(m_leftShooter, m_rightShooter);
    
    m_mainFeeder = new DifferentialDrive(m_topFeeder, m_bottomFeeder);

    m_mainIntake = new DifferentialDrive(m_indexer, m_intake);
  }

  public void shoot(double shoot, double feeder, double indexer, double intake){
    m_mainShoot.tankDrive(shoot, shoot);

    m_mainFeeder.tankDrive(feeder, -feeder);

    m_mainIntake.tankDrive(-indexer, indexer);
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    double[] outputs = {m_indexer.getBusVoltage(),
      m_intake.getBusVoltage()
    };
    SmartDashboard.putNumberArray("outputs", outputs);

  }
}
