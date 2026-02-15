package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ClimbConfigs {
    private SparkMaxConfig climbConfig = new SparkMaxConfig(); //Set name to ensure we can use variable throughout class
    private SparkMaxConfig climbConfig2 = new SparkMaxConfig(); //Set name to ensure we can use variable throughout class


    static {
         climbConfig
            .smartCurrentLimit(50)
            .closedLoop.pid(ClimbPIDConstants.kP, ClimbPIDConstants.kI, ClimbPIDConstants.kD)
            .idleMode(IdleMode.kBrake); //Set limits for leader motor and pid using constant file

        climbConfig2
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake); //Set limits for follower motor

        climbFollowerMotor.follow(climbLeaderMotor); //set follower to follow the leader so we only need to call the leader when moving the climb


    }
}