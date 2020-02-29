/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision{
  /**
   * Creates a new Vision.
   */
  private static NetworkTableInstance table = null;

  /**
   * Light modes for Limelight.
   * 
   * @author Dan Waxman
   */
  public static enum LightMode {
    eOn, eOff, eBlink
  }

  /**
   * Camera modes for Limelight.
   * 
   * @author Dan Waxman
   */
  public static enum CameraMode {
    eVision, eDriver
  }

  /**
   * Stream modes for Limelight.
   * 
   * @author Dan Waxman
   */
  public static enum StreamMode {
    eStandard, eMain, eSecondary
  }

  /**
   * Snapshot modes for Limelight.
   * 
   * @author Dan Waxman
   */
  public static enum SnapshotMode {
    eOff, eOn
  }
  /**
   * Gets whether a target is detected by the Limelight.
   * 
   * @return true if a target is detected, false otherwise.
   */
  public static boolean isTarget() {
    return getValue("tv").getDouble(0) == 1;
  }

  /**
   * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
   * 
   * @return tx as reported by the Limelight.
   */
  public static double getTx() {
    return getValue("tx").getDouble(0.00);
  }

  /**
   * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
   * 
   * @return ty as reported by the Limelight.
   */
  public static double getTy() {
    return getValue("ty").getDouble(0.00);
  }

  /**
   * Area that the detected target takes up in total camera FOV (0% to 100%).
   * 
   * @return Area of target.
   */
  public static double getTa() {
    return getValue("ta").getDouble(0.00);
  }

  /**
   * Gets target skew or rotation (-90 degrees to 0 degrees).
   * 
   * @return Target skew.
   */
  public static double getTs() {
    return getValue("ts").getDouble(0.00);
  }

  /**
   * Gets target latency (ms).
   * 
   * @return Target latency.
   */
  public static double getTl() {
    return getValue("tl").getDouble(0.00);
  }

  /**
   * Sets LED mode of Limelight.
   * 
   * @param mode
   *            Light mode for Limelight.
   */
  public static void setLedMode(LightMode mode) {
    getValue("ledMode").setNumber(mode.ordinal());
  }

  /**
   * Sets camera mode for Limelight.
   * 
   * @param mode
   *            Camera mode for Limelight.
   */
  public static void setCameraMode(CameraMode mode) {
    getValue("camMode").setNumber(mode.ordinal());
  }

  /**
   * Sets stream mode for Limelight.
   * 
   * @param mode
   *            Camera mode for Limelight.
   */
  public static void setStreamMode(StreamMode mode) {
    getValue("stream").setNumber(mode.ordinal());
  }

  /**
   * Sets snapshot mode for Limelight.
   * 
   * @param mode
   *            Camera mode for Limelight.
   */
  public static void setSnapshotMode(SnapshotMode mode) {
    getValue("snapshot").setNumber(mode.ordinal());
  }

  /**
   * Sets pipeline number (0-9 value).
   * 
   * @param number
   *            Pipeline number (0-9).
   */
  public static void setPipeline(int number) {
    getValue("pipeline").setNumber(number);
  }

  /**
   * Helper method to get an entry from the Limelight NetworkTable.
   * 
   * @param key
   *            Key for entry.
   * @return NetworkTableEntry of given entry.
   */
  public static NetworkTableEntry getValue(String key) {
    if (table == null) {
      table = NetworkTableInstance.getDefault();
    }

    return table.getTable("limelight").getEntry(key);
  }

  // d = h/tan(limelight angle +theta)

  // TODO: account for ang displacement of robot (gyro.getPitch)
  public static double getDistance(double angularDisplacement){
    return FieldConstants.RELATIVE_INNER_PORT_HEIGHT/(Math.tan(Math.toRadians(VisionConstants.LIMELIGHT_ANGULAR_DISPLACEMENT_DEGREES + angularDisplacement)));
  }

  // v = (d*sqrt(g))/(cos(theta)*sqrt(2(dtan(theta)-h)))
  public static double getVelocity(double displacement){
    return (displacement * Math.sqrt(9.8)) / (Math.cos(VisionConstants.LIMELIGHT_ANGULAR_DISPLACEMENT_DEGREES) * Math.sqrt(2 * (displacement * Math.tan(VisionConstants.LIMELIGHT_ANGULAR_DISPLACEMENT_DEGREES) - FieldConstants.RELATIVE_INNER_PORT_HEIGHT)));
  }
  //height1 = 65.5 + 18 = 83.5 inches = 212.09 cm + 30.48 = 242.57cm
  //height3 = 19.5 inches = 49.53 cm
  //delta height = 212.09 - 49.53 = 162.56 + 30.48 = 193.04 cm


  public static double getRPM(double velocity){
    return AutoConstants.kVtoRPM * velocity;
  }
}
