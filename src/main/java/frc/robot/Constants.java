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

    public static final int SPINNER = 14;
  }

  public static final class FieldConstants {
    public static double INNER_PORT_HEIGHT = 1.88976; // meters
    public static double LIMELIGHT_HEIGHT = 0; // meters
    public static double RELATIVE_INNER_PORT_HEIGHT = INNER_PORT_HEIGHT - LIMELIGHT_HEIGHT;
  }

  public static final class VisionConstants {
    public static double LIMELIGHT_ANGULAR_DISPLACEMENT_DEGREES = 0;
    public static double LIMELIGHT_ANGULAR_DISPLACEMENT_RADIANS = Math.toRadians(LIMELIGHT_ANGULAR_DISPLACEMENT_DEGREES);
  }

  public static final class AutoConstants {

    // Drive


    // Turn
    public static double TURN_KP = 0.021;
    public static double TURN_KI = 0.0;
    public static double TURN_KD = 0.0;
    public static final double MIN_INPUT = -180.0f;
    public static final double MAX_INPUT = 180.0f;
    public static final double MIN_INGL = -1.0;
    public static final double MAX_INGL = 1.0;
    public static final double TOLERANCE = 0.1;


    // Control Panel
    public static double ROTATION_SPEED = 0.25;
    public static double POSITION_SPEED = 0.15;


    //Shooter
    public static double kVtoRPM = 0;
  }

  public static final class OIConstants {
    public static final int mainStickPort = 0;
    public static final int firstStickPort = 1;
  }
}