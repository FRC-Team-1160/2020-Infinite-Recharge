/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase{
  /**
   * Creates a new DriveTrain.
   */
  private static DriveTrain instance;

  private final CANSparkMax frontLeft,        middleLeft,        backLeft,        frontRight,        middleRight,        backRight;
  private CANEncoder        frontLeftEncoder, middleLeftEncoder, backLeftEncoder, frontRightEncoder, middleRightEncoder, backRightEncoder;

  private final SpeedControllerGroup left, right;
  private final DifferentialDrive mainDrive;
  public AHRS gyro;

  public PIDController turnController;
  public double kP, kI, kD, kF;
  public double minimumInput, maximumInput;
  public double minimumIntegral, maximumIntegral;
  public double kToleranceDegrees;

  public boolean rotateToAngle;
  public double rotateToAngleRate;
  

  private Compressor comp;

  public static DriveTrain getInstance(){
    if (instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public DriveTrain() {

    frontLeft = new CANSparkMax(DriveConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
    middleLeft = new CANSparkMax(DriveConstants.MIDDLE_LEFT_DRIVE, MotorType.kBrushless);
    backLeft = new CANSparkMax(DriveConstants.BACK_LEFT_DRIVE, MotorType.kBrushless);    // leading left spark

    frontLeftEncoder = frontLeft.getEncoder();
    middleLeftEncoder = middleLeft.getEncoder();
    backLeftEncoder = backLeft.getEncoder();
    
    left = new SpeedControllerGroup(frontLeft, middleLeft, backLeft);

    frontRight = new CANSparkMax(DriveConstants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
    middleRight = new CANSparkMax(DriveConstants.MIDDLE_RIGHT_DRIVE, MotorType.kBrushless);
    backRight = new CANSparkMax(DriveConstants.BACK_RIGHT_DRIVE, MotorType.kBrushless);   // leading right spark

    frontRightEncoder = frontRight.getEncoder();
    middleRightEncoder = middleRight.getEncoder();
    backRightEncoder = backRight.getEncoder();

    right = new SpeedControllerGroup(frontRight, middleRight, backRight);

    mainDrive = new DifferentialDrive(left, right);


    gyro = new AHRS(Port.kMXP);

    comp = new Compressor(15);
    comp.start();
  }


    kP = 0.001; 
    kI = 0;
    kD = 0;

    minimumInput = -180.0f;
    maximumInput = 180.0f;
    minimumIntegral = -1.0;
    maximumIntegral = 1.0;
    kToleranceDegrees = 0.5;

    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(minimumInput, maximumInput);
    turnController.setIntegratorRange(minimumIntegral, maximumIntegral);
    turnController.setTolerance(kToleranceDegrees);
    turnController.setSetpoint(0);

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("Min Output", minimumIntegral);
    // SmartDashboard.putNumber("Max Output", maximumIntegral);
    // SmartDashboard.putNumber("Set Angle", 90);
    SmartDashboard.putData(turnController);
  }
  
  public void tankDrive(final double x, final double z, final double correction){
    mainDrive.tankDrive(x-z, x+z); // x is positive when left joystick pulled down
  }

  public void resetYaw(){
    gyro.reset();
  }

  // implements getAsDouble function from Double Supplier (needed for PIDCommand)
  public double getYaw() // AKA getYaw bois
  {
    return gyro.getYaw();
  }

  // implements accept function from Double Consumer (needed for PIDCommand)
  public void accept(double voltage) // moves each gearbox accordingly
  {
    left.set(voltage);
    right.set(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Number: ", gyro.getYaw());
  }
}
