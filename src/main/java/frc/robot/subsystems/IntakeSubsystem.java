// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Configs.IntakeConfig; 

//constructor
public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private SparkMax mIntakeLeader;
    private SparkMax mIntakeFollower;
    private SparkMax mIntakePivotLeader;
    private SparkMax mIntakePivotFollower;

    // Encoders and PID controllers
    private AbsoluteEncoder pivotEncoder;
    private PIDController pivotPID;

    public IntakeSubsystem() {
        // Initialize motors
        mIntakeLeader = new SparkMax(Constants.IntakeConstants.INTAKE_LEADER, MotorType.kBrushless);
        mIntakeFollower = new SparkMax(Constants.IntakeConstants.INTAKE_FOLLOWER, MotorType.kBrushless);
        mIntakePivotLeader = new SparkMax(Constants.IntakeConstants.INTAKE_PIVOT_LEADER, MotorType.kBrushless);
        mIntakePivotFollower = new SparkMax(Constants.IntakeConstants.INTAKE_PIVOT_FOLLOWER, MotorType.kBrushless);

        // Configure motors
        mIntakeLeader.configure(IntakeConfig.intakeLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mIntakeFollower.configure(IntakeConfig.intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mIntakePivotLeader.configure(IntakeConfig.intakePivotLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mIntakePivotFollower.configure(IntakeConfig.intakePivotFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize encoder and PID controller for pivot
        pivotEncoder = mIntakePivotLeader.getAbsoluteEncoder();
        pivotPID = new PIDController(Constants.IntakeConstants.PIVOT_Kp, 
                                     Constants.IntakeConstants.PIVOT_Ki, 
                                     Constants.IntakeConstants.PIVOT_Kd);

        PivotFF = new SimpleMotorFeedforward(Constants.IntakeConstants.PIVOT_FF_kS, Constants.IntakeConstants.PIVOT_FF_kV, 0);
    }


//methods 
public void runIntake(double speed) {
    mIntakeLeader.set(speed);
}

public void runPivot(double targetPosition) {
    // Calculate PID for position
    double pidOutput = pivotPID.calculate(pivotEncoder.getPosition(), targetPosition);

    // Clamp to motor limits
    double speed = MathUtil.clamp(pidOutput, -1.0, 1.0);

    mIntakePivotLeader.set(speed);

    SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Target", targetPosition);
    SmartDashboard.putNumber("Pivot Output", speed);
}


//command to run intake forwards
public Command runIntakeForwards(){
    return run(
    () -> {
        runIntake(Constants.IntakeConstants.INTAKE_FORWARDS);
    });
}

//command to run intake backwards 
public Command runIntakeBackwards(){
    return run(
    () -> {
        runIntake(Constants.IntakeConstants.INTAKE_BACKWARDS);
    });
}

//command to run pivot and deploy the intake PID ONLY
public Command runPivotDeploy(){
    return run(
    () -> {
        runPivot(Constants.IntakeConstants.INTAKE_DEPLOY);
    });
}

//command to run pivot and retract the intake PID ONLY
public Command runPivotRetract(){
    return run(
    () -> {
        runPivot(Constants.IntakeConstants.INTAKE_RETRACT);
    });
}
}