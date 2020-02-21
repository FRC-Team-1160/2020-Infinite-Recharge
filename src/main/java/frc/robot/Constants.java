package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Units are m kg s unless otherwise specified
  public static final class PortConstants {
    // CAN ID 
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int MIDDLE_LEFT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;

    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int MIDDLE_RIGHT_DRIVE = 5;
    public static final int BACK_RIGHT_DRIVE = 6;

    public static final int INTAKE = 7;
    public static final int INDEXER = 8;

    public static final int CLIMBER = 9;

    public static final int LEFT_SHOOTER = 13;
    public static final int RIGHT_SHOOTER = 10;

    public static final int TOP_FEEDER = 12;
    public static final int BOTTOM_FEEDER = 11;

    public static final int INTAKE_ANGLE = 14;

    public static final int SPINNER = 15;
  }

  public static final class DriveConstants{
    public static final double OUTPUT_MIN = 0.1;
    public static final double OUTPUT_MAX = 1;
    public static final double kP_DRIVE = 0.002;
    public static final double ANGLE_THRESHOLD = 0.05;
    public static final double TURN_FACTOR = 0.75;
    public static final double VOLTAGE_TO_SPEED = 4;
  }

  public static final class FieldConstants {
    public static double INNER_PORT_HEIGHT = 2.4257; // meters
    public static double LIMELIGHT_HEIGHT = 0.4953; // meters
    public static double RELATIVE_INNER_PORT_HEIGHT = INNER_PORT_HEIGHT - LIMELIGHT_HEIGHT; //meters; calculation in vision.java
  }

  public static final class VisionConstants {
    public static double LIMELIGHT_ANGULAR_DISPLACEMENT_DEGREES = 15;
    public static double LIMELIGHT_ANGULAR_DISPLACEMENT_RADIANS = Math.toRadians(LIMELIGHT_ANGULAR_DISPLACEMENT_DEGREES);
  }

  public static final class AutoConstants {

    // Drive
    public static double kS = 0;
    public static double kV = 3.59;
    public static double kA = 0.458;
    public static double kRSQUARED = 0.945;
    public static double kTRACKWIDTH = 0.728585410;
    public static double kP_POSITION = 4.13;
    public static double kD_POSITION = 1850.0;
    public static double kP_VELOCITY = 1.8;
    public static double kD_VELOCITY = 0.0;

    // Turn
    public static double TURN_KP = 0.021;
    public static double TURN_KI = 0.0;
    public static double TURN_KD = 0.0;
    public static final double MIN_INPUT = -180.0f;
    public static final double MAX_INPUT = 180.0f;
    public static final double MIN_INGL = -1.0;
    public static final double MAX_INGL = 1.0;
    public static final double TOLERANCE = 0.1;
    public static final double COUNTS_PER_REV_SPARK = 42;
    public static final double COUNTS_PER_REV_775 = 1024;



    // Control Panel


    //Shooter
    public static double kVtoRPM = 0;
  }

  public static final class OIConstants {
    public static final int mainStickPort = 0;
    public static final int firstStickPort = 1;
    public static final int secondStickPort = 2;
    public static final int thirdStickPort = 3;

  }
}