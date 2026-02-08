package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public final class Configs {
    public static final class ShooterConfig {
        public static final SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFeederConfig = new SparkMaxConfig();

        static {
            // Leader configuration
            shooterLeaderConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast);

            // Follower configuration - set to follow Leader (ID 14) and invert
            shooterFollowerConfig
                .follow(ShooterConstants.SHOOTER_LEADER_CANID, true)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast);

            // Feeder/Kicker configuration
            shooterFeederConfig
                .smartCurrentLimit(30);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkMaxConfig intakeLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();

        static {
            intakeLeaderConfig
                .smartCurrentLimit(40);

            intakeFollowerConfig
                .follow(IntakeConstants.INTAKE_LEADER_ID, true)
                .smartCurrentLimit(40);
        }
    }
}