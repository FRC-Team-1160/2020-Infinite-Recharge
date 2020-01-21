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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase{
  /**
   * Creates a new DriveTrain.
   */
  private static DriveTrain instance;

  private final CANSparkMax frontLeft,        middleLeft,        backLeft,        frontRight,        middleRight,        backRight;
  // private CANEncoder        frontLeftEncoder, middleLeftEncoder, backLeftEncoder, frontRightEncoder, middleRightEncoder, backRightEncoder;

  private final DifferentialDrive mainDrive;
  public AHRS gyro;

  public PIDController turnController;

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
    backLeft = new CANSparkMax(DriveConstants.BACK_LEFT_DRIVE, MotorType.kBrushless);    

    frontLeft.follow(backLeft);
    middleLeft.follow(backLeft);
    
    frontRight = new CANSparkMax(DriveConstants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
    middleRight = new CANSparkMax(DriveConstants.MIDDLE_RIGHT_DRIVE, MotorType.kBrushless);
    backRight = new CANSparkMax(DriveConstants.BACK_RIGHT_DRIVE, MotorType.kBrushless);   

    frontRight.follow(backRight);
    middleRight.follow(backRight);

    mainDrive = new DifferentialDrive(backLeft, backRight);

    gyro = new AHRS(Port.kMXP);

    comp = new Compressor(15);
    comp.start();
 
    // SmartDashboard.putData("PDP", new PowerDistributionPanel(15));

    turnController = new PIDController(AutoConstants.TURN_KP, AutoConstants.TURN_KI, AutoConstants.TURN_KD);
    turnController.enableContinuousInput(AutoConstants.MIN_INPUT, AutoConstants.MAX_INPUT);
    turnController.setIntegratorRange(AutoConstants.MIN_INGL, AutoConstants.MAX_INGL);
    turnController.setTolerance(AutoConstants.TOLERANCE);
    turnController.setSetpoint(0);

    SmartDashboard.putData("Turn Controller", turnController);
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