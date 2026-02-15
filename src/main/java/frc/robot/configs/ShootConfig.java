package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public final class ShootConfig {
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
                //.follow(ShooterConstants.SHOOTER_LEADER_CANID, true)
                .inverted(true)
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast);

            // Feeder/Kicker configuration
            shooterFeederConfig
                .smartCurrentLimit(30);
        }
    }
