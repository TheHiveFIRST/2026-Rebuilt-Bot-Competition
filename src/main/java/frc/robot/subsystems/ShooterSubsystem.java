package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.ShootConfig.ShooterConfig;

import static edu.wpi.first.units.Units.*;

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
    private boolean mDistanceEstimation = true;
    private boolean mShooterEnabled = false;

    private final MutVoltage m_appliedVoltage  = Volts.mutable(0);
    private final MutAngle m_encoderAngle    = Rotations.mutable(0);
    private final MutAngularVelocity m_encoderVelocity = RotationsPerSecond.mutable(0);

    private final SysIdRoutine mSysIdRoutine;

    private String shotType = "HUB_SHOT";


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

        mSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),   // ramp rate  – 1 V/s (quasistatic)
                Volts.of(4),               // step voltage – 4 V  (dynamic)
                Seconds.of(5)              // timeout per direction
            ),
            new SysIdRoutine.Mechanism(
                // Drive: apply a raw voltage to both shooter motors
                voltage -> {
                    double v = voltage.in(Volts);
                    mShooterLeader  .setVoltage(v);
                    mShooterFollower.setVoltage(v);
                },
                // Log: record voltage, position, and velocity for the leader
                log -> {
                    log.motor("shooter-leader")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                mShooterLeader.getBusVoltage()
                                    * mShooterLeader.getAppliedOutput(),
                                Volts))
                        .angularPosition(
                            m_encoderAngle.mut_replace(
                                mShooterLeaderEncoder.getPosition(),
                                Rotations))
                        .angularVelocity(
                            m_encoderVelocity.mut_replace(
                                // Encoder reports RPM – convert to RPS for WPILib units
                                mShooterLeaderEncoder.getVelocity() / 60.0,
                                RotationsPerSecond));
                },
                this   // owning subsystem (for requirement tracking)
            )
        );

    }

    public void setShooterSpeeds(double setRPM, double RPMOffset) {
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        double pidOutput = mShooterPID.calculate(mCurrentRPM, setRPM + RPMOffset);
        double ffOutput = tempFF.calculate(setRPM + RPMOffset); 
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        runShooterPower(motorPower); 
    }

    public double getShooterPIDF(double setRPM, double RPMOffset) {
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        double pidOutput = mShooterPID.calculate(mCurrentRPM, setRPM + RPMOffset);
        double ffOutput = tempFF.calculate(setRPM + RPMOffset); 
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        return motorPower; 
    }

   
   
    //OLD TESTING ONE REVERT BACK IF IT DOESNT WORK 
    public void runDirectShooterPIDF(double setRPM) {
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        double targetRPM = setRPM; 
        double pidOutput = mShooterPID.calculate(mCurrentRPM, targetRPM);
        double ffOutput = tempFF.calculate(targetRPM); 
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);

        //TODO: test with different feedforward, boosting target RPM  
        // double ffVelocityOutput = tempFF.calculateWithVelocities(mCurrentRPS,mTargetRPS);
        // Temporarily boost target RPM when shooting
       // if (mCurrentRPM < mTargetRPM - 400) {
       //     mTargetRPM = mTargetRPM + 200; // Preemptive compensation
       // }
 

    }

    public void runShooterPower(double motorPower){
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
    }
    public void setSpeedsSmartDashboard(){
    setShooterSpeeds(SmartDashboard.getNumber("Testing/setShooterRPM", 300), 0);
    }


    public void runShooterForDistance(double distance){
        double shooterRegressionRPM = 
          (Math.pow(distance, 3) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_3)
        + (Math.pow(distance, 2) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_2)
        + (Math.pow(distance, 1) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_1)
        +(Constants.ShooterConstants.REGRESSION_COEFFICIENT_0);
        updateRPM(shooterRegressionRPM); 
    }

     // Finds the average velocity of the two motors 
    public double getAverageVelocity() {
        double sum = mShooterLeaderEncoder.getVelocity() + mShooterFollowerEncoder.getVelocity();
        double average = sum / 2;
        return average;
    }
    private void changeShootingRPMOffset(double amount){
      ShooterRPMOffset += amount;
    } 
  


    public void stopAll() {
            runShooterPower(0);
            runKicker(0);
        }

    public void runKicker(double speed){
            mKickerLeader.set(speed);
            mKickerFollower.set(-1 * speed);
        }
    public void incrementRPM() { mTargetRPM += ShooterConstants.RPM_INCREMENT; }
    public void decrementRPM() { mTargetRPM -= ShooterConstants.RPM_INCREMENT; }

    public void updateRPM(double newRPM){
        mTargetRPM = newRPM; 
    
    }
    

    //Commands 
   // public Command runShooterCommand() { return run(this::runShooterPIDF); }
    // public Command runShooterCommand() {
    //      return run(
    //     () -> {
    //         runDirectShooterPIDF(mTargetRPM);
    //           });
    // }
    
    public Command runShooterAutoCommand() {       
        return run(
        () -> {
            setShooterSpeeds(ShooterConstants.AUTO_TARGET_RPM, 0);
              }); 
        }
    
    public Command runShooterPIDFCommand() {
         return run(
        () -> {
            setShooterSpeeds(mTargetRPM, ShooterRPMOffset);
              });
    }
     public Command runShooterRegressionCommand() {
         return run(
        () -> {
            runShooterForDistance(DriveSubsystem.hubDistance);
              });
    }

    public Command toggleShooterCommand() {
        return new InstantCommand(() -> mShooterEnabled = !mShooterEnabled);}

    public Command toggleDistanceEstimationCommand() {
        return new InstantCommand(() -> mDistanceEstimation = !mDistanceEstimation);}
    
    public Command toggleAutoShooterCommand() {
       return new InstantCommand(() -> mShooterEnabled = true);}
    
    public Command toggleOffAutoShooterCommand() {
       return new InstantCommand(() -> mShooterEnabled = false);}
    
    
    
    public Command runShooterPowerCommand() {
         return run(
        () -> {
            runShooterPower(ShooterConstants.SHOOTER_SPEED);
              });
    }

    
    public Command increaseShootingRPMOffsetCommand(){
    return new InstantCommand(() -> changeShootingRPMOffset(ShooterConstants.RPMOFFSET_INCREMENT));
    }
    public Command decreaseShootingRPMOffsetCommand(){
      return new InstantCommand(() -> changeShootingRPMOffset(-ShooterConstants.RPMOFFSET_INCREMENT));
    }
    public Command setHubShotCommand() {
    return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.HUB_TARGET_RPM;
        ShooterRPMOffset = 0;
        mDistanceEstimation = false; 
        shotType = "BUMPER_ALIGN_SHOT";
        });
    }
public Command setAutoShotCommand() {
    return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.AUTORPM;
        ShooterRPMOffset = 0;
        shotType = "AUTOSHOT";
        });
    }

    public Command setTrenchShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.TRENCH_TARGET_RPM;
        ShooterRPMOffset = 0;
        shotType = "TRENCH_SHOT";

        });}
    
    public Command setDefenceShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.DEFENCE_TARGET_RPM;
        ShooterRPMOffset = 0;
        shotType = "DEFENCE_SHOT";

        });}

    public Command setLadderShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.LADDER_TARGET_RPM;
        ShooterRPMOffset = 0;
        shotType = "TOWER_SHOT";

        });}
    
    public Command setPassingShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.PASSING_TARGET_RPM;
        ShooterRPMOffset = 0;
        shotType = "PASSING_SHOT";
        });}



    public Command stop() {
         return run(
        () -> {
            stopAll();
              });
    }

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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return mSysIdRoutine.quasistatic(direction);
    }
 
    /**
     * Dynamic test – applies a voltage step.
     * Used to fit kA.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return mSysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        if(mDistanceEstimation == true){        
        runShooterForDistance(DriveSubsystem.hubDistance);
        } else{
        updateRPM(mTargetRPM);
        }

        if (mShooterEnabled == true) {
        setShooterSpeeds(mTargetRPM, ShooterRPMOffset);
        } else {
        runShooterPower(0);
        }

        SmartDashboard.putNumber("Shooter/Target RPM", mTargetRPM +ShooterRPMOffset);
        SmartDashboard.putNumber("Shooter/Actual RPM", mShooterLeaderEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/RPM Offset", ShooterRPMOffset);
        SmartDashboard.putString("Shooter/Shot Type", shotType);
        //SmartDashboard.putNumber("Testing/shooter current", mShooterLeader.getOutputCurrent());
        //SmartDashboard.putNumber("Testing/shooter motor 2 current", mShooterFollower.getOutputCurrent());
        boolean atSpeed = Math.abs(mShooterLeaderEncoder.getVelocity() - mTargetRPM + ShooterRPMOffset) < ShooterConstants.VELOCITY_TOLERANCE;
        SmartDashboard.putBoolean("Shooter/Shooter Ready", atSpeed);
        SmartDashboard.putBoolean("Shooter/Shooter Toggled", mShooterEnabled);
        SmartDashboard.putBoolean("Shooter/Regression Toggled", mDistanceEstimation);
    }

     // Tuning  
    // 
    //public enum TuningMode {
    //     KP, KV, KD, 
    // }
    // private TuningMode mCurrentTuningMode = TuningMode.KV;
    // // Cycle through tuning modes
    // public void cycleTuningMode() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             mCurrentTuningMode = TuningMode.KP;
    //             break;
    //         case KV:
    //             mCurrentTuningMode = TuningMode.KV;
    //             break;
    //         case KD:
    //             mCurrentTuningMode = TuningMode.KD;
    //             break;
    //     }
    // }
    //     public void incrementCurrentGain() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             incrementKP();
    //             break;
    //         case KV:
    //             incrementKV();
    //             break;
    //         case KD:
    //             incrementKD();
    //             break;
    //     }
    // }
    //     public void decrementCurrentGain() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             decrementKP();
    //             break;
    //         case KV:
    //             decrementKV();
    //             break;
    //         case KD:
    //             decrementKD();
    //             break;
    //     }
    // }

    // public void incrementKD() { mCurrentKD += ShooterConstants.KD_INCREMENT; } 
    // public void decrementKD() { mCurrentKD -= ShooterConstants.KD_INCREMENT; }

    // public void incrementKI() { mCurrentKI += ShooterConstants.KI_INCREMENT; } 
    // public void decrementKI() { mCurrentKI -= ShooterConstants.KI_INCREMENT; }

    // public void incrementKP() { mCurrentKP += ShooterConstants.KP_INCREMENT; } 
    // public void decrementKP() { mCurrentKP -= ShooterConstants.KP_INCREMENT; }

    // public void incrementKV() { mCurrentKV += ShooterConstants.KV_INCREMENT; } 
    // public void decrementKV() { mCurrentKV -= ShooterConstants.KV_INCREMENT; }
}