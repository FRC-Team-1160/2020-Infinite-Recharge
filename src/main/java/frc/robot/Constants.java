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
    public static final class DriveConstants {
        // CAN ID 
        public static final int FRONT_LEFT_DRIVE = 1;
        public static final int MIDDLE_LEFT_DRIVE = 2;
        public static final int BACK_LEFT_DRIVE = 3;

        public static final int FRONT_RIGHT_DRIVE = 4;
        public static final int MIDDLE_RIGHT_DRIVE = 5;
        public static final int BACK_RIGHT_DRIVE = 6;

        public static final int LEFT_SHOOTER = 7;
        public static final int RIGHT_SHOOTER = 8;
    }
  
    public static final class HatchConstants {
      
    }
  
    public static final class AutoConstants {
      public static double TURN_KP = 0.021;
      public static double TURN_KI = 0.0;
      public static double TURN_KD = 0.0;
      public static double MIN_INPUT = -180.0f;
      public static double MAX_INPUT = 180.0f;
      public static double MIN_INGL = -1.0;
      public static double MAX_INGL = 1.0;
      public static double TOLERANCE = 0.1;
    }
  
    public static final class OIConstants {
      public static final int mainStickPort = 0;
    }
  }