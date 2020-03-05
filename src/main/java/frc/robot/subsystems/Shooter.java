/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANPIDController;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;


public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static Shooter m_instance;

  private final CANSparkMax  m_leftShooter; // m_rightShooter;

  private CANEncoder m_leftEncoder; // m_rightEncoder;

  private CANPIDController m_shootController;

  private PIDController m_RIOshootController;

  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }
  
  public Shooter() {
    if (Constants.isFinal){
      m_leftShooter = new CANSparkMax(PortConstantsFinal.RIGHT_SHOOTER, MotorType.kBrushed);
      // m_leftShooter = new CANSparkMax(PortConstantsFinal.LEFT_SHOOTER, MotorType.kBrushed);
      // m_rightShooter = new CANSparkMax(PortConstantsFinal.RIGHT_SHOOTER, MotorType.kBrushed);

    }else{
      m_leftShooter = new CANSparkMax(PortConstants.LEFT_SHOOTER, MotorType.kBrushed);
     //  m_rightShooter = new CANSparkMax(PortConstants.RIGHT_SHOOTER, MotorType.kBrushed);
    }


    m_leftShooter.restoreFactoryDefaults();
    // m_rightShooter.restoreFactoryDefaults();

    // m_rightShooter.follow(m_leftShooter, false);

    // m_leftEncoder = m_leftShooter.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1024);
    m_leftEncoder = m_leftShooter.getEncoder(EncoderType.kQuadrature, 1024);

    // m_rightEncoder = m_rightShooter.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1024);

    m_shootController = m_leftShooter.getPIDController();

    // m_shootController.setFeedbackDevice(m_leftEncoder);


    // PID coefficients
    kP = 1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    m_shootController.setP(kP);
    m_shootController.setI(kI);
    m_shootController.setD(kD);
    m_shootController.setIZone(kIz);
    m_shootController.setFF(kFF);
    m_shootController.setOutputRange(kMinOutput, kMaxOutput);

    /*
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Set Rotations", 0);
    */

  }

  public void shooterControl(double input){
    m_leftShooter.setVoltage(-input);
    // m_shootController.setReference(5, ControlType.kVoltage);
    // m_shootController.setReference(1000, ControlType.kPosition);
  }

  /*public void PIDShooterPositionControl(double input){
    m_shootController.setReference(input, ControlType.kPosition);
  }*/

  @Override
  public void periodic() {
    /*
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    */
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    /*if((p != kP)) { m_shootController.setP(p); kP = p; }
    if((i != kI)) { m_shootController.setI(i); kI = i; }
    if((d != kD)) { m_shootController.setD(d); kD = d; }
    if((iz != kIz)) { m_shootController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_shootController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_shootController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }*/

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    //m_shootController.setReference(rotations, ControlType.kPosition);
    
    // SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_leftEncoder.getPosition());

  }
}