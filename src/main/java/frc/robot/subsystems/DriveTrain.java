/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase{
  /**
   * Creates a new DriveTrain.
   */
  private static DriveTrain m_instance;

  private final CANSparkMax m_frontLeft,        m_middleLeft,        m_backLeft,        m_frontRight,        m_middleRight,        m_backRight;
  // private CANEncoder        frontLeftEncoder, middleLeftEncoder, backLeftEncoder, frontRightEncoder, middleRightEncoder, backRightEncoder;

  double d;

  private final DifferentialDrive m_mainDrive;

  public AHRS m_gyro;

  public PIDController m_turnController;

  private Compressor m_comp;

  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  public DriveTrain() {

    m_frontLeft = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
    m_middleLeft = new CANSparkMax(DriveConstants.MIDDLE_LEFT_DRIVE, MotorType.kBrushless);
    m_backLeft = new CANSparkMax(DriveConstants.BACK_LEFT_DRIVE, MotorType.kBrushless);    

    m_frontLeft.follow(m_backLeft);
    m_middleLeft.follow(m_backLeft);
    
    m_frontRight = new CANSparkMax(DriveConstants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
    m_middleRight = new CANSparkMax(DriveConstants.MIDDLE_RIGHT_DRIVE, MotorType.kBrushless);
    m_backRight = new CANSparkMax(DriveConstants.BACK_RIGHT_DRIVE, MotorType.kBrushless);   

    m_frontRight.follow(m_backRight);
    m_middleRight.follow(m_backRight);

    m_mainDrive = new DifferentialDrive(m_backLeft, m_backRight);

    m_gyro = new AHRS(Port.kMXP);

    m_comp = new Compressor(15);
    m_comp.start();
 
    d = 10;
    SmartDashboard.putNumber("Input d:", d);

    m_turnController = new PIDController(AutoConstants.TURN_KP, AutoConstants.TURN_KI, AutoConstants.TURN_KD);
    m_turnController.enableContinuousInput(AutoConstants.MIN_INPUT, AutoConstants.MAX_INPUT);
    m_turnController.setIntegratorRange(AutoConstants.MIN_INGL, AutoConstants.MAX_INGL);
    m_turnController.setTolerance(AutoConstants.TOLERANCE);
    m_turnController.setSetpoint(0);

    SmartDashboard.putData("Turn Controller", m_turnController);
  }
  
  public void tankDrive(final double x, final double z, final double correction){
    m_mainDrive.tankDrive(x-z, x+z); // x is positive when left joystick pulled down
  }

  public void resetYaw(){
    m_gyro.reset();
  }

  public double getYaw() {
    return m_gyro.getYaw();
  }

  public double getPitch(){
    return m_gyro.getPitch();
  }

  public double getHeight(){
    double a1 = getPitch();
    double a2 = Vision.getTy();
    double angle = Math.toRadians(a2+a1);
    double height = d*Math.tan(angle);
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("tan", Math.tan(angle));
    SmartDashboard.putNumber("height", height);
    return height;
  }

  public void accept(double voltage) // moves each gearbox accordingly
  {
    voltage = Math.max(-0.5, Math.min(voltage, 0.5));
    m_backLeft.setVoltage(voltage);
    m_backRight.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    d = SmartDashboard.getNumber("Input d:", 0.0);
    getHeight();
  }
}
