// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.PersistMode; 
import com.revrobotics.ResetMode; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Configs.IntakeConfig; 



public class IntakeSubsystem extends SubsystemBase{

//variables 
private SparkMax mIntakeLeader; 
private SparkMax mIntakeFollower; 
private SparkMax mIntakePivotLeader; 
private SparkMax mIntakePivotFollower; 
private SparkMax mShooterFeeder;
    
//constructor
public IntakeSubsystem(){
    
    //create intake motors + pivotmotors 
    mIntakeLeader = new SparkMax(Constants.IntakeConstants.INTAKE_LEADER, MotorType.kBrushless);
    mIntakeFollower = new SparkMax(Constants.IntakeConstants.INTAKE_FOLLOWER, MotorType.kBrushless);
    mIntakePivotLeader = new SparkMax(Constants.IntakeConstants.INTAKE_PIVOT_LEADER, MotorType.kBrushless);
    mIntakePivotFollower = new SparkMax(Constants.IntakeConstants.INTAKE_PIVOT_FOLLOWER, MotorType.kBrushless);
    mShooterFeeder = new SparkMax(Constants.IntakeConstants.SHOOTER_FEEDER, MotorType.kBrushless);

    //apply configuration to motors 
    mIntakeLeader.configure(IntakeConfig.intakeLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mIntakeFollower.configure(IntakeConfig.intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mIntakePivotLeader.configure(IntakeConfig.intakePivotLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mIntakePivotFollower.configure(IntakeConfig.intakePivotFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mShooterFeeder.configure(IntakeConfig.ShooterFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}


//methods 
public void runIntake(double speed) {
    mIntakeLeader.set(speed);
}

public void runPivot(double speed) {
    mIntakePivotLeader.set(speed);
}
 
public void feedShooter(double speed) {
    mShooterFeeder.set(speed);
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

//command to run pivot and deploy the intake 
public Command runPivotDeploy(){
    return run(
    () -> {
        runPivot(Constants.IntakeConstants.INTAKE_DEPLOY);
    });
}

//command to run pivot and retract the intake 
public Command runPivotRetract(){
    return run(
    () -> {
        runPivot(Constants.IntakeConstants.INTAKE_RETRACT);
    });
}
//command to feed fuel into the shooter
public Command feedShooterForward(){
    return run(
    () -> {
        feedShooter(Constants.IntakeConstants.FEEDER_FEED);
    });
}

//command to run feeder backwards to unjam it or smth
public Command evacuateShooter(){
    return run(
    () -> {
        feedShooter(Constants.IntakeConstants.FEEDER_EVACUATE);
    });

}
}
// hi.