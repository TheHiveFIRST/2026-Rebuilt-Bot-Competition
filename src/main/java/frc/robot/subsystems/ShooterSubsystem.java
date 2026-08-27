package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.ShootConfig.ShooterConfig;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax mShooterLeader;
    private final SparkMax mShooterFollower;
    private final SparkMax mKickerLeader;
    private final SparkMax mKickerFollower;


    private final RelativeEncoder mShooterLeaderEncoder;
    private final RelativeEncoder mShooterFollowerEncoder; 
    private final PIDController mShooterPID;
    private final SimpleMotorFeedforward tempFF;

    private double mTargetRPM = ShooterConstants.HUB_TARGET_RPM;

    private double ShooterRPMOffset = 0; 
    
    private boolean mShooterEnabled = false;

    


    public ShooterSubsystem() {
    
        mShooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
        mShooterFollower = new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
        mKickerLeader = new SparkMax(ShooterConstants.SHOOTER_FEEDER_LEADER_CANID, MotorType.kBrushless);
        mKickerFollower = new SparkMax(ShooterConstants.SHOOTER_FEEDER_FOLLOWER_CANID, MotorType.kBrushless);


        mShooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mKickerLeader.configure(ShooterConfig.shooterFeederLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mKickerFollower.configure(ShooterConfig.shooterFeederFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterLeaderEncoder = mShooterLeader.getEncoder();
        mShooterFollowerEncoder = mShooterFollower.getEncoder();

        mShooterPID = new PIDController(ShooterConstants.LEADER_Kp, 
        ShooterConstants.LEADER_Ki, ShooterConstants.LEADER_Kd);
        
        tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS,
        ShooterConstants.LEADER_FF_kV, 
        ShooterConstants.LEADER_FF_kA);
    }
//shooter operation methods
    public void runShooterPower(double motorPower){
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
    }

    public void setShooterSpeeds(double setRPM, double RPMOffset) {
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        double pidOutput = mShooterPID.calculate(mCurrentRPM, setRPM + RPMOffset);
        double ffOutput = tempFF.calculate(setRPM + RPMOffset); 
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        runShooterPower(motorPower); 
    }
//Shooter calculations
     // Finds the average velocity of the two motors 
    public double getAverageVelocity() {
        double sum = mShooterLeaderEncoder.getVelocity() + mShooterFollowerEncoder.getVelocity();
        double average = sum / 2;
        return average;
    }
    private void changeShootingRPMOffset(double amount){
      ShooterRPMOffset += amount;
    } 
  

//kicker operation methods
    public void stopAll() {
            runShooterPower(0);
            runKicker(0);
        }

    public void runKicker(double speed){
            mKickerLeader.set(speed);
            mKickerFollower.set(-1 * speed);
        }
  
//Shooter operation commands
    public Command toggleShooterCommand() {
        return new InstantCommand(() -> mShooterEnabled = !mShooterEnabled);}

    public Command increaseShootingRPMOffsetCommand(){
    return new InstantCommand(() -> changeShootingRPMOffset(ShooterConstants.RPMOFFSET_INCREMENT));
    }
    public Command decreaseShootingRPMOffsetCommand(){
      return new InstantCommand(() -> changeShootingRPMOffset(-ShooterConstants.RPMOFFSET_INCREMENT));
    }
   
//Kicker operation commands
    public Command runKickerCommand() {
         return run(
        () -> {
            runKicker(-ShooterConstants.KICKER_SPEED);
        
              });
    }

    public Command runKickerBackwardCommand() {
         return run(
        () -> {
            runKicker(ShooterConstants.KICKER_SPEED);
            
              });
    }

//stop command
    public Command stop() {
         return run(
        () -> {
            stopAll();
              });
    }


    @Override
    public void periodic() {
        
        if (mShooterEnabled == true) {
        setShooterSpeeds(mTargetRPM, ShooterRPMOffset);
        } else {
        runShooterPower(0);
        }

        SmartDashboard.putNumber("Shooter/Target RPM", mTargetRPM +ShooterRPMOffset);
        SmartDashboard.putNumber("Shooter/Actual RPM", mShooterLeaderEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/RPM Offset", ShooterRPMOffset);
        
        
        boolean atSpeed = Math.abs(mShooterLeaderEncoder.getVelocity() - mTargetRPM + ShooterRPMOffset) < ShooterConstants.VELOCITY_TOLERANCE;
        SmartDashboard.putBoolean("Shooter/Shooter Ready", atSpeed);
        SmartDashboard.putBoolean("Shooter/Shooter Toggled", mShooterEnabled);
        
    }

   
}