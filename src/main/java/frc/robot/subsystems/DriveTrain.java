/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private static DriveTrain instance;

  private final Talon fL, fR, bL, bR;

  private final SpeedControllerGroup left, right;

  private final DifferentialDrive mainDrive;

  private final AHRS gyro;

  public static DriveTrain getInstance(){
    if (instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public DriveTrain() {
    fL = new Talon(DriveConstants.FRONT_LEFT_DRIVE);
    bL = new Talon(DriveConstants.BACK_LEFT_DRIVE);

    left = new SpeedControllerGroup(fL, bL);
    
    fR = new Talon(DriveConstants.FRONT_RIGHT_DRIVE);
    bR = new Talon(DriveConstants.BACK_RIGHT_DRIVE);

    right = new SpeedControllerGroup(fR, bR);

    mainDrive = new DifferentialDrive(left, right);

    gyro = new AHRS(Port.kMXP);
  }

  /*
  public void drive(double left, double right){
    fL.set(left);
    bL.set(left);
    fR.set(right);
    bR.set(right);
    System.out.println(left);
    System.out.println(right);
  }
  */

  public void tankDrive(final double x, final double z, final double correction){
    mainDrive.tankDrive(-x+z, -x-z); // x is positive when left joystick pulled down
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
    SmartDashboard.putNumber("angle", getAngle());
  }
}
