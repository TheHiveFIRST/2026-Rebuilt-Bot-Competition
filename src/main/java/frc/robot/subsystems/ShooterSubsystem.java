package frc.robot.subsystems;

import com.revrobotics.PersistMode; 
import com.revrobotics.ResetMode; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.configs.ShooterConfig;
import frc.robot.Constants; 





public class ShooterSubsystem extends SubsystemBase {
    
//variables 
private SparkMax mShooterRightLeader; 
private SparkMax mShooterRightFollower; 
private SparkMax mShooterLeftLeader; 
private SparkMax mShooterLeftFollower; 

//constructor 
public ShooterSubsystem() {
    
    //create motors for flywheels motors
    mShooterRightLeader = new SparkMax(Constants.ShooterConstants.SHOOTER_RIGHT_LEADER_CANID, MotorType.kBrushless);
    mShooterLeftLeader = new SparkMax(Constants.ShooterConstants.SHOOTER_LEFT_LEADER_CANID, MotorType.kBrushless);
    mShooterRightFollower = new SparkMax(Constants.ShooterConstants.SHOOTER_RIGHT_FOLLOWER_CANID, MotorType.kBrushless);
    mShooterLeftFollower = new SparkMax(Constants.ShooterConstants.SHOOTER_LEFT_FOLLOWER_CANID, MotorType.kBrushless);

    //apply configurations to flywheel motors
    mShooterRightLeader.configure(ShooterConfig.shooterRightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mShooterRightFollower.configure(ShooterConfig.shooterRightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mShooterLeftLeader.configure(ShooterConfig.shooterLeftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mShooterLeftFollower.configure(ShooterConfig.shooterLeftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

//methods 
public void runShooter(double speed) {
    mShooterRightLeader.set(speed);
    mShooterLeftLeader.set(speed); 
}

//command to run shooter forwards
public Command runShooterForwards() 
{
    return run(
    () -> {
        runShooter(Constants.ShooterConstants.SHOOT_FORWARDS);
    });
}

//command to run shooter backwards

public Command runShooterBackwards()
{
    return run(
    () -> {
        runShooter(Constants.ShooterConstants.SHOOT_BACKWARDS);
    });
}








}
