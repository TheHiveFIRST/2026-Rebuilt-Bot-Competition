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

  private SparkMax climbLeaderMotor;
  private SparkMax climbFollowerMotor;
  private SparkMaxConfig climbConfig = new SparkMaxConfig();
  private SparkMaxConfig climbConfig2 = new SparkMaxConfig();

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    climbLeaderMotor = new SparkMax(ClimbConstants.climbLeaderCanID, MotorType.kBrushless);
    climbFollowerMotor = new SparkMax(ClimbConstants.climbFollowerCanID, MotorType.kBrushless);

    climbConfig
      .smartCurrentLimit(50)
      .closedLoop.pid(ClimbPIDConstants.kP, ClimbPIDConstants.kI, ClimbPIDConstants.kD)
      .idleMode(IdleMode.kBrake);

    climbLeaderMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbLeaderMotor.setPositionConversionFactor(ClimbPIDConstants.conversionFactor);

    climbConfig2
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

    climbFollowerMotor.configure(climbConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climbFollowerMotor.follow(climbLeaderMotor);
  }

  public void setStartClimb() {
    climbLeaderMotor.getClosedLoopController().setReference(ClimbConstants.endPose, ControlType.kPosition);
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
          setStartClimb();
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

