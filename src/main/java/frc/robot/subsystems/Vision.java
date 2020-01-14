/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  private NetworkTable limelight;

  private static Vision instance;

  public static Vision getInstance(){
    if (instance == null){
      instance = new Vision();
    }
    return instance;
  }

  public Vision() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    limelight.getEntry("ledMode").setNumber(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
