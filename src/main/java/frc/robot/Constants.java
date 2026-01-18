// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_RIGHT_LEADER_CANID = 1;
    public static final int SHOOTER_RIGHT_FOLLOWER_CANID = 2;
    public static final int SHOOTER_LEFT_LEADER_CANID = 3;
    public static final int SHOOTER_LEFT_FOLLOWER_CANID = 4;
    public static final double SHOOT_FORWARDS = 5600;
    public static final double SHOOT_BACKWARDS = -5600;

    // PIDF values
    public static final double RIGHT_Kp = 0.01;
    public static final double RIGHT_Ki = 0;
    public static final double RIGHT_Kd = 0;

    public static final double LEFT_Kp = 0.01;
    public static final double LEFT_Ki = 0;
    public static final double LEFT_Kd = 0;

    public static final double RIGHT_FF_kS = 1.787;
    public static final double RIGHT_FF_kV = 0.4;

    public static final double LEFT_FF_kS = 1.787;
    public static final double LEFT_FF_kV = 0.4;
  }

  public static class IntakeConstants {
    public static final int INTAKE_LEADER = 0;
    public static final int INTAKE_FOLLOWER = 1; 
    public static final int INTAKE_PIVOT_LEADER = 2; 
    public static final int INTAKE_PIVOT_FOLLOWER = 3; 
    public static final int SHOOTER_FEEDER = 4; 

    public static final double FEEDER_FEED = .5; //these are all placeholders
    public static final double FEEDER_EVACUATE = .5; //these are all placeholders
    public static final double INTAKE_FORWARDS = .5; 
    public static final double INTAKE_BACKWARDS = .5; 
    public static final double INTAKE_DEPLOY = .5; 
    public static final double INTAKE_RETRACT = .5; 
  }
}
