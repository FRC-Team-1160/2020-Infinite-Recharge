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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static Shooter m_instance;

  private final CANSparkMax  m_leftShooter, m_rightShooter;
  private CANEncoder         m_leftEncoder, m_rightEncoder;

  private CANPIDController speedController;

  //private final DifferentialDrive m_mainShoot;

  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, velocity;

  private boolean run;

  public static Shooter getInstance(){
    if (m_instance == null){
      m_instance = new Shooter();
    }
    return m_instance;
  }
  
  public Shooter() {
    m_leftShooter = new CANSparkMax(PortConstants.LEFT_SHOOTER, MotorType.kBrushless);
    m_rightShooter = new CANSparkMax(PortConstants.RIGHT_SHOOTER, MotorType.kBrushless);
    
    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();

    // inverse of left motor
    m_rightShooter.follow(m_leftShooter, true);

    m_leftEncoder = m_leftShooter.getEncoder();
    m_rightEncoder = m_rightShooter.getEncoder();

    // should be the same as right motor
    speedController = m_leftShooter.getPIDController();

    speedController.setFeedbackDevice(m_leftEncoder);

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    speedController.setP(kP);
    speedController.setI(kI);
    speedController.setD(kD);
    speedController.setIZone(kIz);
    speedController.setFF(kFF);
    speedController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Velocity", 0);

    //m_mainShoot = new DifferentialDrive(m_leftShooter, m_rightShooter);
  }

  public void shooterControl(double velocity){
    if (velocity > 0.0) speedController.setReference(velocity, ControlType.kVelocity);
    //m_mainShoot.tankDrive(input, input);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    velocity = SmartDashboard.getNumber("Set Velocity", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { speedController.setP(p); kP = p; }
    if((i != kI)) { speedController.setI(i); kI = i; }
    if((d != kD)) { speedController.setD(d); kD = d; }
    if((iz != kIz)) { speedController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { speedController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
        speedController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
    }

    // if (run)
    // {
    //   speedController.setReference(velocity, ControlType.kVelocity);
    // }

    SmartDashboard.putNumber("SetPoint", velocity);
    SmartDashboard.putNumber("ProcessVariable", m_leftEncoder.getVelocity());
  }
}
