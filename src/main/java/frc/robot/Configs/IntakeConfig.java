package frc.robot.Configs;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;


public class IntakeConfig {
    public static final SparkMaxConfig intakeLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakePivotLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakePivotFollowerConfig = new SparkMaxConfig();
    

    static {
         //set current limits for flywheels 
        intakeLeaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);
        intakeFollowerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
        .inverted(true);
        intakePivotLeaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);
        intakePivotFollowerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
        .inverted(true);
       
       
        
        intakeFollowerConfig.follow(Constants.IntakeConstants.INTAKE_LEADER);
        intakePivotFollowerConfig.follow(Constants.IntakeConstants.INTAKE_PIVOT_LEADER);
    }
}

