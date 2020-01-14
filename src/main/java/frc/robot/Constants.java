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
        //Talon Rio Ports
        public static final int FRONT_LEFT_DRIVE = 1;
        public static final int MIDDLE_LEFT_DRIVE = 2;
        public static final int BACK_LEFT_DRIVE = 3;

        public static final int FRONT_RIGHT_DRIVE = 4;
        public static final int MIDDLE_RIGHT_DRIVE = 5;
        public static final int BACK_RIGHT_DRIVE = 6;

    }
  
    public static final class HatchConstants {
      
    }
  
    public static final class AutoConstants {
      public static double TURN_ANGLE = 90;
     
    }
  
    public static final class OIConstants {
      public static final int mainStickPort = 0;
    }
  }