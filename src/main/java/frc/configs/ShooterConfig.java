package frc.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ShooterConfig {
    public static final SparkMaxConfig shooterLeftLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterRightLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterLeftFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterRightFollowerConfig = new SparkMaxConfig();

    static {
         //set current limits for flywheels 
        shooterRightLeaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);
        shooterRightFollowerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);
        shooterLeftLeaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);
        shooterLeftFollowerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);

        shooterRightLeaderConfig.follow(Constants.ShooterConstants.SHOOTER_RIGHT_LEADER_CANID);
        shooterLeftLeaderConfig.follow(Constants.ShooterConstants.SHOOTER_LEFT_LEADER_CANID);
    }
}
