package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class ShooterConstants {
        // CAN IDs
        public static final int SHOOTER_LEADER_CANID = 14;
        public static final int SHOOTER_FOLLOWER_CANID = 15;
        public static final int SHOOTER_FEEDER = 16; 

        // PIDF Values (Tuned for your shooter)
        public static final double LEADER_Kp = 0.00061;
        public static final double LEADER_Kd = 0.00003;
        public static final double LEADER_Ki = 0.000019998;
        public static final double LEADER_FF_kS = 0.0;
        public static final double LEADER_FF_kV = 0.0112;
        public static final double LEADER_FF_kA = 0.0002;
    
        // Manual Control Settings
        public static final double RPM_INCREMENT = 50.0;
        public static final double KD_INCREMENT = 0.00001;
        public static final double KP_INCREMENT = 0.00001;
        public static final double KI_INCREMENT = 0.00001;


        public static final double DEFAULT_TARGET_RPM = 5500.0;
        public static final double VELOCITY_TOLERANCE = 100; 
        public static final double KICKER_SPEED = 1.0;
        public static final double SHOOTER_SPEED = 0.1;

    }

    public static class IntakeConstants {
        public static final int INTAKE_LEADER_ID = 10;
        public static final int INTAKE_FOLLOWER_ID = 11;
        
        // Speeds
        public static final double INTAKE_SPEED = 0.7;
    }
}