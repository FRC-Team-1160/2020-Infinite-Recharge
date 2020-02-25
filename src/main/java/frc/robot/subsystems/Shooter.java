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

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static Shooter m_instance;

  private final CANSparkMax  m_leftShooter, m_rightShooter;

  private final DifferentialDrive m_mainShoot;

  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }
  
  public Shooter() {
    m_leftShooter = new CANSparkMax(PortConstants.LEFT_SHOOTER, MotorType.kBrushed);
    m_rightShooter = new CANSparkMax(PortConstants.RIGHT_SHOOTER, MotorType.kBrushed);

    
    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();

    m_mainShoot = new DifferentialDrive(m_leftShooter, m_rightShooter);
  }

  public void shooterControl(double input){
    m_mainShoot.tankDrive(-input, input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}