package frc.robot.configs;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;

public class IntakeConfigs {
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
