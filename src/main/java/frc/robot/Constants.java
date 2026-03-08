package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity (length, complexity).
 */
public final class Constants {
  public static final class DriveConstants{ 
    //allowed max speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 9.6; // TODO: after test change back to 4.8 and 2pi;
    public static final double MAX_ANGULAR_SPEED =  2.5*Math.PI; // rad/s 
    //Chassis config - width, depth, CAN IDS and angular offset values in Designdoc.md
  
    public static final double WHEEL_CENTER_WIDTH = Units.inchesToMeters(11.75);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_CENTER_DEPTH = Units.inchesToMeters(11.75);
    // Distance between front and back wheels on robot
    //depth/frontback distance from robot center to each wheel 

    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_CENTER_DEPTH, WHEEL_CENTER_WIDTH),
        new Translation2d(WHEEL_CENTER_DEPTH, -WHEEL_CENTER_WIDTH),
        new Translation2d(-WHEEL_CENTER_DEPTH, WHEEL_CENTER_WIDTH),
        new Translation2d(-WHEEL_CENTER_DEPTH, -WHEEL_CENTER_WIDTH));
      
    //angular offsets of module relative to chassis (rad)
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI/2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = -Math.PI/2;
    
    //SPARK MAX CAN IDs 
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 4;
    public static final int FRONT_LEFT_TURNING_CAN_ID = 3;

    
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 2;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 1;
    
    public static final int BACK_LEFT_DRIVING_CAN_ID = 6;
    public static final int BACK_LEFT_TURNING_CAN_ID = 5;
   
    public static final int BACK_RIGHT_DRIVING_CAN_ID = 8;
    public static final int BACK_RIGHT_TURNING_CAN_ID = 7;

    public static final boolean GYRO_REVERSED = false;
    
    public static final double INTAKE_ALIGN_KP = 0.0001;
    public static final double DIAGONAL_ALIGN_kP = 0.0001;

    public static double DRIVE_SPEED = 1;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;
    public static final double AUTO_ALIGN_PID = 0.047;
    public static final double AUTO_ALIGN_MAX_SPEED = 1.00;


  }

  public static final class ModuleConstants{
    // The MAXSwerve module 3 pinion ggears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (more teeth = faster)
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = MotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

  }
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
    public static final double DRIVE_DEADBAND = 0.05; 
  }

  public static final class AutoConstants {
    //add constants here that are not in pathplanner/limelight if needed
    public static final double X_TAG_ALIGNMENT_P = 0.01; //TODO: tune limelight pid  
    public static final double Y_TAG_ALIGNMENT_P = 0.1; 
    public static final double ROT_TAG_ALIGNMENT_P = 0.1; 

    public static final double ROT_SETPOINT_TAG_ALIGNMENT = 0;  //  RY Rotation
    public static final double ROT_TOLERANCE_TAG_ALIGNMENT = 1;
    
    public static final double X_SETPOINT_TAG_ALIGNMENT = 9.79;  //tx Vertical pose and tolerance 
    public static final double X_TOLERANCE_TAG_ALIGNMENT = 0.02;

    public static final double Y_SETPOINT_TAG_ALIGNMENT = -0.19;  // tz Horizontal pose (- for diff sides of tag)
    public static final double Y_TOLERANCE_TAG_ALIGNMENT = 0.02; 

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;
  }

  public static final class VisionConstants{
   public static final double LL_MOUNT_ANGLE_DEG = 0; //a1: degrees rotated up from vertical 
   public static final double LL_LENS_HEIGHT_IN = 7.1; //h1: distance from lens to floor 
   public static final double TARGET_HEIGHT_IN = 12.5;//44.25; //h2: height of target 
  }

  public static final class MotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  } 
  
  public static final class IntakeConstants {
        public static final int INTAKE_LEADER_ID = 10; 
        public static final int INTAKE_FOLLOWER_ID = 11;
        public static final double INTAKE_SPEED = 1;
        public static final double SLOW_INTAKE_SPEED = 0.05;
        public static final double OUTTAKE_SPEED = -0.5;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_LEADER_CANID = 14;
        public static final int SHOOTER_FOLLOWER_CANID = 15;
        public static final int SHOOTER_FEEDER = 16; 

        // PIDF Values
        public static final double LEADER_Kp = 0.0000709999; // 0.00061;
        public static final double LEADER_Kd = 0; //0.00003;
        public static final double LEADER_Ki = 0; //0.000019998;
        public static final double LEADER_FF_kS = 0.0;
        public static final double LEADER_FF_kV = 0.00012;
        public static final double LEADER_FF_kA = 0.0002;

        public static final double TESTING_KP = 0.0006;
        public static final double TESTING_KI = 0;
        public static final double TESTING_KD = 0;
        public static final double FF_KS = 0.0;
        public static final double FF_KV = 0.0;
        public static final double FF_KA = 0.0;
    
        // Manual Control 
        public static final double RPM_INCREMENT = 50.0;
        public static final double KV_INCREMENT = 0.000001;
        public static final double KP_INCREMENT = 0.001;
        public static final double KI_INCREMENT = 0.0001;
        public static final double KD_INCREMENT = 0.0001;

        public static final double KICKER_SPEED = 1.0;
        public static final double SHOOTER_SPEED = 0.7; 

        public static final double VELOCITY_TOLERANCE =  30; 

        public static final double HUB_TARGET_RPM = 4800; //TUNED 
        public static final double TRENCH_TARGET_RPM = 6800; 
        public static final double LADDER_TARGET_RPM = 6300;
        public static final double PASSING_TARGET_RPM = 8000;
        public static final double DEFENCE_TARGET_RPM = 5200; 

        public static final double AUTO_TARGET_RPM = 5400; 

        public static final double RPMOFFSET_INCREMENT = 200; 

        //Untested Regression coefficients 
        public static final double REGRESSION_COEFFICIENT_4 = 0; 
        public static final double REGRESSION_COEFFICIENT_3 = 0;
        public static final double REGRESSION_COEFFICIENT_2 = 0;
        public static final double REGRESSION_COEFFICIENT_1 = 0;
        public static final double REGRESSION_COEFFICIENT_0 = 0;

    }

    public static final class ArmConstants {
        public static final int ARM_LEADER_ID = 12;
        public static final int ARM_FOLLOWER_ID = 13;

        // PID Gains
        public static final double ARM_KP = 0.94; 
        public static final double ARM_KI = 0.0;
        public static final double ARM_KD = 0.0;

        public static final double PIVOT_OUT = 0.56; 
        public static final double PIVOT_DEFAULT = 0.408; 
        public static final double PIVOT_OUT_FAHH = 0.53; 

        public static final double PIVOT_IN = 0.184;  
        public static final double ARM_KP_INCREMENT = 0.01;

        public static final double POSITION_TOLERANCE = 0.02;
    }

}

   

