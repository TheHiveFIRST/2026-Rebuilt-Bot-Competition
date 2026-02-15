// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax.ControlType;
import com.revrobotics.spark.SparkMax.ResetMode;
import com.revrobotics.spark.SparkMax.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import Constants.ClimbConstants;
import Constants.ClimbPIDConstants;
import frc.robot.configs.ClimbConfigs;

public class ClimbSubsystem extends SubsystemBase {

  private SparkMax climbLeaderMotor; //Set name to ensure we can use variable throughout class
  private SparkMax climbFollowerMotor; //Set name to ensure we can use variable throughout class

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    climbLeaderMotor = new SparkMax(ClimbConstants.climbLeaderCanID, MotorType.kBrushless); //Create a new motor (brushless)
    climbFollowerMotor = new SparkMax(ClimbConstants.climbFollowerCanID, MotorType.kBrushless); //Create a new motor (brushless)

    climbLeaderMotor.configure(ClimbConfigs.climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //set configuration of leadermotor
    climbLeaderMotor.setPositionConversionFactor(ClimbConfigs.ClimbPIDConstants.conversionFactor); //Convert from rotations of robot to inchs

    climbFollowerMotor.configure(ClimbConfigs.climbConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //set configuration of followermotor

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

