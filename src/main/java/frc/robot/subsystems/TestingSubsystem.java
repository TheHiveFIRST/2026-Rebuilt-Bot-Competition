// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class TestingSubsystem extends SubsystemBase {
  // --- Hardware Definitions ---
  private final SparkMax m_intakeLeader = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax m_intakeFollower = new SparkMax(11, MotorType.kBrushless);
  private final SparkMax m_shooterLeft = new SparkMax(14, MotorType.kBrushless);
  private final SparkMax m_shooterRight = new SparkMax(15, MotorType.kBrushless);
  private final SparkMax m_kicker = new SparkMax(16, MotorType.kBrushless);

  /** Creates a new TestingSubsystem. */
  public TestingSubsystem() {
    configureHardware();
  }

  private void configureHardware() {
    SparkMaxConfig baseConfig = new SparkMaxConfig();
    baseConfig.smartCurrentLimit(40); 

    SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();
    intakeFollowerConfig.follow(10, true);

    SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
    shooterFollowerConfig.follow(14, true);

    SparkMaxConfig kickerConfig = new SparkMaxConfig();
    kickerConfig.inverted(true);

    // Apply configurations
    m_intakeLeader.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeFollower.configure(intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterLeft.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterRight.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // --- Methods (Actions) ---

  public void setIntakeSpeed(double speed) {
    m_intakeLeader.set(speed);
  }

  public void setShooterSpeed(double speed) {
    m_shooterLeft.set(speed);
  }

  public void setKickerSpeed(double speed) {
    m_kicker.set(speed);
  }

  public void stopAll() {
    m_intakeLeader.set(0);
    m_shooterLeft.set(0);
    m_kicker.set(0);
  }

  // --- Command Factories ---

  /** Simple command to run the kicker while held. */
  public Command runKickerCommand(double speed) {
    return startEnd(() -> setKickerSpeed(speed), () -> setKickerSpeed(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run. 
    // Great for putting data to SmartDashboard.
  }
}