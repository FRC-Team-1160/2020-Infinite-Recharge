/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private Talon fL, fR, bL, bR;

  public DriveTrain() {
    fL = new Talon(DriveConstants.FRONT_LEFT_DRIVE);
    fR = new Talon(DriveConstants.FRONT_RIGHT_DRIVE);
    bL = new Talon(DriveConstants.BACK_LEFT_DRIVE);
    bR = new Talon(DriveConstants.BACK_RIGHT_DRIVE);
  }

  public void drive(double left, double right){
    fL.set(left);
    bL.set(left);
    fR.set(right);
    bR.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
