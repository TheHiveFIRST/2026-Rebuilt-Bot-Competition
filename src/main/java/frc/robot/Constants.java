package frc.robot;

public final class Constants {

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_LEADER_ID = 10; // Check your real IDs!
        public static final int INTAKE_FOLLOWER_ID = 11;
        public static final double INTAKE_SPEED = 0.7;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_LEADER_CANID = 14;
        public static final int SHOOTER_FOLLOWER_CANID = 15;
        public static final int SHOOTER_FEEDER = 16;
        
        public static final double DEFAULT_TARGET_RPM = 2000;
        public static final double VELOCITY_TOLERANCE = 100;
        
        // Gains
        public static final double LEADER_Kp = 0.0001;
        public static final double LEADER_Ki = 0.0;
        public static final double LEADER_Kd = 0.0;
        public static final double LEADER_FF_kS = 0.1;
        public static final double LEADER_FF_kV = 0.002;
        public static final double LEADER_FF_kA = 0.0001;
        
        public static final double SHOOTER_SPEED = 0.8;
        public static final double KICKER_SPEED = 0.5;
        
        public static final double RPM_INCREMENT = 100;
        public static final double KP_INCREMENT = 0.00001;
        public static final double KI_INCREMENT = 0.00001;
        public static final double KD_INCREMENT = 0.00001;
    }

    public static final class ArmConstants {
        public static final int kArmLeaderId = 12;
        public static final int kArmFollowerId = 13;

        // PID Gains
        public static final double kP = 0.5; 
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Positions (0 to 1 for Absolute Encoders)
        public static final double kPivotOut = 0.4; 
        public static final double kPivotIn = 0.1;  
        
        public static final double kPositionTolerance = 0.02;
    }
}