package frc.robot;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 1;
    }

    public static class ShooterConstants {
        public static final int SHOOTER_LEADER_CANID = 14;
        public static final int SHOOTER_FOLLOWER_CANID = 15;
        public static final int KICKER_CANID = 16; 

        // PIDF Values
        public static final double LEADER_Kp = 0.01;
        public static final double LEADER_FF_kS = 0.1;
        public static final double LEADER_FF_kV = 0.4;
        
        // Settings
        public static final double RPM_INCREMENT = 10.0;
        public static final double DEFAULT_TARGET_RPM = 2000.0;
    }

    public static class IntakeConstants {
        public static final int INTAKE_LEADER_ID = 10;
        public static final int INTAKE_FOLLOWER_ID = 11;
        public static final double INTAKE_SPEED = 0.7;
    }
}