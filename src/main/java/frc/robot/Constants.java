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
        public static final int SHOOTER_TESTER_LEADER_CANID = 17; //TODO: delete later
        public static final int SHOOTER_TESTER_FOLLOWER_CANID = 18;
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
        public static final double KP_INCREMENT = 0.00001;
        public static final double KI_INCREMENT = 0.00001;
        public static final double KD_INCREMENT = 0.00001;

        public static final double KICKER_SPEED = 0.8;
        public static final double SHOOTER_SPEED = 0.2; 

        public static final double VELOCITY_TOLERANCE =  30; 

        public static final double DEFAULT_TARGET_RPM = 5000; 

    }

    public static final class ArmConstants {
        public static final int ARM_LEADER_ID = 12;
        public static final int ARM_FOLLOWER_ID = 13;

        // PID Gains
        public static final double ARM_KP = 0.5; 
        public static final double ARM_KI = 0.0;
        public static final double ARM_KD = 0.0;

        // Positions (0 to 1 for Absolute Encoders)
        public static final double PIVOT_OUT = 0.4; 
        public static final double PIVOT_IN = 0.1;  
        
        public static final double POSITION_TOLERANCE = 0.02;
    }
}