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
    public static final int kDriverControllerPort = 0;
  }
  public static class ClimbConstants {
    public static final int climbLeaderCanID = 11; //ID for climb leader
    public static final int climbFollowerCanID = 12; //ID for climb leader
    public static final double endPose = 0.5; //Where to extend the climb so it latches onto the end game piece
    public static final double startPose = 0; //Reset the climb position to the original position so it lifts the robot
  }
  public static class ClimbPIDConstants {
    public static final double kP = 0.001; //PID controls for leader climber
    public static final double kI = 0.001; //PID controls for leader climber
    public static final double kD = 0.001; //PID controls for leader climber
    public static final double conversionFactor = 1; //Converstion factor from rotations to inchs
  }
}
