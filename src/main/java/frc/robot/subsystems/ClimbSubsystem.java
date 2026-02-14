// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxConfig;
import com.revrobotics.spark.SparkMax.ControlType;
import com.revrobotics.spark.SparkMax.ResetMode;
import com.revrobotics.spark.SparkMax.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import Constants.ClimbConstants;
import Constants.ClimbPIDConstants;

public class ClimbSubsystem extends SubsystemBase {

  private SparkMax climbLeaderMotor; //Set name to ensure we can use variable throughout class
  private SparkMax climbFollowerMotor; //Set name to ensure we can use variable throughout class
  private SparkMaxConfig climbConfig = new SparkMaxConfig(); //Set name to ensure we can use variable throughout class
  private SparkMaxConfig climbConfig2 = new SparkMaxConfig(); //Set name to ensure we can use variable throughout class

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    climbLeaderMotor = new SparkMax(ClimbConstants.climbLeaderCanID, MotorType.kBrushless); //Create a new motor (brushless)
    climbFollowerMotor = new SparkMax(ClimbConstants.climbFollowerCanID, MotorType.kBrushless); //Create a new motor (brushless)

    climbConfig
      .smartCurrentLimit(50)
      .closedLoop.pid(ClimbPIDConstants.kP, ClimbPIDConstants.kI, ClimbPIDConstants.kD)
      .idleMode(IdleMode.kBrake); //Set limits for leader motor and pid using constant file

    climbLeaderMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //set configuration of leadermotor
    climbLeaderMotor.setPositionConversionFactor(ClimbPIDConstants.conversionFactor); //Convert from rotations of robot to inchs

    climbConfig2
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake); //Set limits for follower motor

    climbFollowerMotor.configure(climbConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //set configuration of followermotor

    climbFollowerMotor.follow(climbLeaderMotor); //set follower to follow the leader so we only need to call the leader when moving the climb
  }

  public void setStartClimb() {
    climbLeaderMotor.getClosedLoopController().setReference(ClimbConstants.endPose, ControlType.kPosition); //Sets climb position to the desirecd extension
  }
  public void setEndClimb() {
    climbLeaderMotor.getClosedLoopController().setReference(ClimbConstants.startPose, ControlType.kPosition); //Moves climb back to initialized position
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command extendClimb() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setStartClimb(); //Calls the funtion to extend the climb
        });
  }
  public Command retractClimb() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setEndClimb(); //Calls the function to lift the robot
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

