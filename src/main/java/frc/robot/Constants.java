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

        // PIDF Values
        public static final double LEADER_Kp = 0.000001; // 0.00061;
        public static final double LEADER_Kd = 0; //0.00003;
        public static final double LEADER_Ki = 0; //0.000019998;
        public static final double LEADER_FF_kS = 0.0;
        public static final double LEADER_FF_kV = 0.000183;
        public static final double LEADER_FF_kA = 0.0002;

        public static final double TESTING_KP = 0.0006;
        public static final double TESTING_KI = 0;
        public static final double TESTING_KD = 0;
        public static final double FF_KS = 0.0;
        public static final double FF_KV = 0.0;
        public static final double FF_KA = 0.0;
    
        // Manual Control Settings
        public static final double RPM_INCREMENT = 50.0;
        public static final double KV_INCREMENT = 0.000001;
        public static final double KP_INCREMENT = 0.000001;
        public static final double KI_INCREMENT = 0.0001;
        public static final double KD_INCREMENT = 0.0001;

        public static final double KICKER_SPEED = 0.8;
        public static final double SHOOTER_SPEED = 0.7; 

        public static final double VELOCITY_TOLERANCE =  30; 

        public static final double DEFAULT_TARGET_RPM = 2000; 

    }

    public static final class ArmConstants {
        public static final int ARM_LEADER_ID = 12;
        public static final int ARM_FOLLOWER_ID = 13;

        // PID Gains
        public static final double ARM_KP = 0.32; 
        public static final double ARM_KI = 0.0;
        public static final double ARM_KD = 0.0;

        // Positions (0 to 1 for Absolute Encoders)
        public static final double PIVOT_OUT = 0.400; 
        public static final double PIVOT_IN = 0.3;  
        
        public static final double POSITION_TOLERANCE = 0.02;
    }
}