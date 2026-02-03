package frc.robot.Configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ShooterConfig {
    public static final SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig shooterFeederConfig = new SparkMaxConfig();

    public static final SparkMaxConfig encoderConfig = new SparkMaxConfig();

    static {
         //set current limits for flywheels 
        shooterLeaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);
        shooterFollowerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .follow(Constants.ShooterConstants.SHOOTER_LEADER_CANID);
        
        shooterFeederConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .inverted(true);

        shooterFollowerConfig.follow(Constants.ShooterConstants.SHOOTER_LEADER_CANID);


    }
}
