package frc.robot.configs;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class DriveConfig {
    public static final class MAXSwerveModule{
        public static final SparkMaxConfig frontLeftDrivingConfig = new SparkMaxConfig(); 
        public static final SparkMaxConfig frontRightDrivingConfig = new SparkMaxConfig(); 
        public static final SparkMaxConfig backLeftDrivingConfig = new SparkMaxConfig(); 
        public static final SparkMaxConfig backRightDrivingConfig = new SparkMaxConfig(); 

        public static final SparkMaxConfig frontLeftTurningConfig = new SparkMaxConfig(); 
        public static final SparkMaxConfig frontRightTurningConfig = new SparkMaxConfig(); 
        public static final SparkMaxConfig backLeftTurningConfig = new SparkMaxConfig(); 
        public static final SparkMaxConfig backRightTurningConfig = new SparkMaxConfig(); 


        static {
            //use module constance to calc conversion factors + feed forward gain 
            double drivingFactor = ModuleConstants.WHEEL_DIAMETER_METERS * Math.PI
                    /ModuleConstants.DRIVING_MOTOR_REDUCTION;
            double turningFactor = 2* Math.PI;  
            double drivingVelocityFeedForward = 1 / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_RPS;

            //Current limit should ALWAYS be below 50 for driving and 20 for turning 
            frontLeftDrivingConfig
            .idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            frontLeftDrivingConfig.encoder
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor/60.0);
            frontLeftDrivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.FRONT_LEFT_DRIVING_P, 0,ModuleConstants.FRONT_LEFT_DRIVING_D)  
            .outputRange(-1,1)  //speed setpoint to actual velocity  
            .feedForward.kV(drivingVelocityFeedForward);
            

            frontRightDrivingConfig
            .idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            frontRightDrivingConfig.encoder
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor/60.0);
            frontRightDrivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.FRONT_RIGHT_DRIVING_P, 0,ModuleConstants.FRONT_RIGHT_DRIVING_D)  
            .outputRange(-1,1)  //speed setpoint to actual velocity  
            .feedForward.kV(drivingVelocityFeedForward);


            backLeftDrivingConfig
            .idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            backLeftDrivingConfig.encoder
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor/60.0);
            backLeftDrivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.BACK_LEFT_DRIVING_P, 0,ModuleConstants.BACK_LEFT_DRIVING_D)  
            .outputRange(-1,1)  //speed setpoint to actual velocity  
            .feedForward.kV(drivingVelocityFeedForward);


            backRightDrivingConfig
            .idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            backRightDrivingConfig.encoder
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor/60.0);
            backRightDrivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.BACK_RIGHT_DRIVING_P, 0,ModuleConstants.BACK_RIGHT_DRIVING_D)  
            .outputRange(-1,1)  //speed setpoint to actual velocity  
            .feedForward.kV(drivingVelocityFeedForward);
            backRightTurningConfig
            .idleMode(IdleMode.kBrake).smartCurrentLimit(20);
            
            
            frontLeftTurningConfig.absoluteEncoder
            //invert as output shaft rotates oppositie to steering motor in MAXSwerveModule
            .inverted(true)  
            .positionConversionFactor(turningFactor) //radians 
            .velocityConversionFactor(turningFactor/60.0) //rad/s 
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
            frontLeftTurningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ModuleConstants.FRONT_LEFT_TURNING_P, 0,0)  
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);

            frontRightTurningConfig.absoluteEncoder
            //invert as output shaft rotates oppositie to steering motor in MAXSwerveModule
            .inverted(true)  
            .positionConversionFactor(turningFactor) //radians 
            .velocityConversionFactor(turningFactor/60.0) //rad/s 
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
            frontRightTurningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ModuleConstants.FRONT_RIGHT_TURNING_P, 0,0)  
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);


            backLeftTurningConfig.absoluteEncoder
            //invert as output shaft rotates oppositie to steering motor in MAXSwerveModule
            .inverted(true)  
            .positionConversionFactor(turningFactor) //radians 
            .velocityConversionFactor(turningFactor/60.0) //rad/s 
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
            backLeftTurningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ModuleConstants.BACK_LEFT_TURNING_P, 0,0)  
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);

            backRightTurningConfig.absoluteEncoder
            //invert as output shaft rotates oppositie to steering motor in MAXSwerveModule
            .inverted(true)  
            .positionConversionFactor(turningFactor) //radians 
            .velocityConversionFactor(turningFactor/60.0) //rad/s 
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
            backRightTurningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ModuleConstants.BACK_RIGHT_TURNING_P, 0,0)  
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);
        }


    }
}