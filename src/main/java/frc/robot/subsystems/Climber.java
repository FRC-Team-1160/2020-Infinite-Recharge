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
import frc.robot.Constants.PortConstants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */  
  private static Climber m_instance;

  private CANSparkMax m_climber;

  public static Climber getInstance(){
    if (m_instance == null){
      m_instance = new Climber();
    }
    return m_instance;
  }

  public Climber() {
    m_climber = new CANSparkMax(PortConstants.CLIMBER, MotorType.kBrushless);

    m_climber.restoreFactoryDefaults();
  }

  public void climbControl(double speed){
    m_climber.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
