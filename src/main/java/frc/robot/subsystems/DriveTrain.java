/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private static DriveTrain instance;

  private final CANSparkMax frontLeft,        middleLeft,        backLeft,        frontRight,        middleRight,        backRight;
  private CANEncoder        frontLeftEncoder, middleLeftEncoder, backLeftEncoder, frontRightEncoder, middleRightEncoder, backRightEncoder;

  private final SpeedControllerGroup left, right;
  private final DifferentialDrive mainDrive;
  private AHRS gyro;
  private Joystick stick;

  public PIDController turnController;
  private CANPIDController leftPIDController;
  public double kP, kI, kD, kF;
  public double minimumInput, maximumInput;
  public double minimumIntegral, maximumIntegral;
  public double kToleranceDegrees;

  public boolean rotateToAngle;
  public double rotateToAngleRate;
  

  public static DriveTrain getInstance(){
    if (instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public DriveTrain() {

    frontLeft = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
    middleLeft = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
    backLeft = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);    // leading left spark

    frontLeftEncoder = frontLeft.getEncoder(EncoderType.kQuadrature, 4096);
    middleLeftEncoder = middleLeft.getEncoder(EncoderType.kQuadrature, 4096);
    backLeftEncoder = backLeft.getEncoder(EncoderType.kQuadrature, 4096);
    
    left = new SpeedControllerGroup(frontLeft, middleLeft, backLeft);

    frontRight = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
    middleRight = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
    backRight = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);   // leading right spark

    frontRightEncoder = frontRight.getEncoder(EncoderType.kQuadrature, 4096);
    middleRightEncoder = middleRight.getEncoder(EncoderType.kQuadrature, 4096);
    backRightEncoder = backRight.getEncoder(EncoderType.kQuadrature, 4096);

    right = new SpeedControllerGroup(frontRight, middleRight, backRight);

    mainDrive = new DifferentialDrive(left, right);

    try {
      gyro = new AHRS(Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    stick = new Joystick(0);

    kP = 0.1; 
    kI = 1e-4;
    kD = 1;

    minimumInput = -180.0f;
    maximumInput = 180.0f;
    minimumIntegral = -1.0;
    maximumIntegral = 1.0;
    kToleranceDegrees = 0.5;

    turnController = new PIDController(kP, kI, kD);
    turnController.setIntegratorRange(minimumIntegral, maximumIntegral);
    turnController.setTolerance(kToleranceDegrees);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Min Output", minimumIntegral);
    SmartDashboard.putNumber("Max Output", maximumIntegral);
    SmartDashboard.putNumber("Set Angle", 90);
  }

  public void setFollowers(CANSparkMax follower1, CANSparkMax follower2, CANSparkMax leader)
  {
    follower1.follow(leader);
    follower2.follow(leader);
  }
  
  public void tankDrive(final double x, final double z, final double correction){
    mainDrive.tankDrive(x-z, x+z); // x is positive when left joystick pulled down
  }

  public double getAngle(){
    return gyro.getYaw();
  }

  public void resetAngle(){
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboards
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double angle = SmartDashboard.getNumber("Set Angle", 0);

    // if PID coefficients on SmartDashboard have changed, write new values
    if((p != kP)) { kP = p; }
    if((i != kI)) { kI = i; }
    if((d != kD)) { kD = d; }

    boolean rotateToAngle = false;
    if (stick.getRawButton(1)) {
      gyro.reset();
    }
    if (stick.getRawButton(2)) {
      turnController.setSetpoint(0.0f);
      rotateToAngle = true;
    } else if (stick.getRawButton(3)) {
      turnController.setSetpoint(90.0f);
      rotateToAngle = true;
    } else if (stick.getRawButton(4)) {
      turnController.setSetpoint(179.9f);
      rotateToAngle = true;
    } else if (stick.getRawButton(5)) {
      turnController.setSetpoint(-90.0f);
      rotateToAngle = true;
    }

    if (rotateToAngle) {
      turnController.enableContinuousInput(minimumInput, maximumInput);
      rotateToAngleRate = turnController.calculate(gyro.getYaw());
      left.set(rotateToAngleRate);
      right.set(-rotateToAngleRate);
    } else {
      turnController.disableContinuousInput();
      rotateToAngleRate = stick.getTwist();
    }
  }
}
