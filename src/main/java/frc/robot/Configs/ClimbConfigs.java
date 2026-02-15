package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ClimbPIDConstants;

import frc.robot.Constants;

public class ClimbConfigs {
    public static final SparkMaxConfig climbConfig = new SparkMaxConfig(); //Set name to ensure we can use variable throughout class
    public static final SparkMaxConfig climbConfig2 = new SparkMaxConfig(); //Set name to ensure we can use variable throughout class


    static {
         climbConfig
            .smartCurrentLimit(40)
            .closedLoop.pid(ClimbPIDConstants.kP, ClimbPIDConstants.kI, ClimbPIDConstants.kD)
            .idleMode(IdleMode.kCoast); //Set limits for leader motor and pid using constant file

        climbConfig2
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast); //Set limits for follower motor

        climbConfig2.follow(Constants.ClimbConstants.climbLeaderCanID); //set follower to follow the leader so we only need to call the leader when moving the climb
    }
}