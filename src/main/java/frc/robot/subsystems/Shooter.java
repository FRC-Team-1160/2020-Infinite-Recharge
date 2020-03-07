/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstantsFinal;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static Shooter m_instance;

  private final TalonSRX m_leftShooter, m_rightShooter;

  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }
  
  public Shooter() {
    if (Constants.isFinal){
      m_leftShooter = new TalonSRX(PortConstantsFinal.LEFT_SHOOTER);
      m_rightShooter = new TalonSRX(PortConstantsFinal.RIGHT_SHOOTER);
    }else{
      m_leftShooter = new TalonSRX(PortConstantsFinal.LEFT_SHOOTER);
      m_rightShooter = new TalonSRX(PortConstantsFinal.RIGHT_SHOOTER);
    }

    m_rightShooter.follow(m_leftShooter, FollowerType.PercentOutput);

    m_leftShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,	ShooterConstants.slotIDx, 30);	

    m_leftShooter.config_kP(ShooterConstants.slotIDx, ShooterConstants.kP, ShooterConstants.kTimeoutMs);
		m_leftShooter.config_kI(ShooterConstants.slotIDx, ShooterConstants.kI, ShooterConstants.kTimeoutMs);
		m_leftShooter.config_kD(ShooterConstants.slotIDx, ShooterConstants.kD, ShooterConstants.kTimeoutMs);
		// m_leftShooter.config_kF(0, ShooterConstants.kF, Constants.kTimeoutMs);
  }

  public void shooterControl(double input){
    m_leftShooter.set(ControlMode.PercentOutput, -input/12);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder Position", m_leftShooter.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Shooter Encoder Velocity", m_leftShooter.getSelectedSensorVelocity(0));
  }
}