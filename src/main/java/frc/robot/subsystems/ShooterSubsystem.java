package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.ShootConfig.ShooterConfig;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax mShooterLeader;
    private final SparkMax mShooterFollower;
    private final SparkMax mKicker;


    private final RelativeEncoder mShooterLeaderEncoder;
    private final RelativeEncoder mShooterFollowerEncoder; 
    private final PIDController mShooterPID;
    private final BangBangController mShooterBangBang;

    
    // --- State Variables ---
    private double mTargetRPM = ShooterConstants.DEFAULT_TARGET_RPM;
    private double mCurrentKV = ShooterConstants.LEADER_FF_kV;
    private double mCurrentKA = ShooterConstants.LEADER_FF_kA;
    private double mCurrentKP = ShooterConstants.LEADER_Kp;
    private double mCurrentKI = ShooterConstants.LEADER_Ki;
    private double mCurrentKD = ShooterConstants.LEADER_Kd;



    public ShooterSubsystem() {
    
        mShooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
        mShooterFollower = new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
        mKicker = new SparkMax(ShooterConstants.SHOOTER_FEEDER, MotorType.kBrushless);


        mShooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mKicker.configure(ShooterConfig.shooterFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mShooterLeaderEncoder = mShooterLeader.getEncoder();
        mShooterFollowerEncoder = mShooterFollower.getEncoder();

        mShooterPID = new PIDController(mCurrentKP, mCurrentKI, mCurrentKD);

        mShooterBangBang = new BangBangController();
        mShooterBangBang.setTolerance(30); 

        
    }

    public void runShooterPIDF() {
        // We create a temporary FF object using the live mCurrentKV variable
        SimpleMotorFeedforward tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS, mCurrentKV, mCurrentKA);

            
        mShooterPID.setP(mCurrentKP);
        mShooterPID.setI(mCurrentKI);
        mShooterPID.setD(mCurrentKD);
        
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        
        double mTargetRPS = mTargetRPM / 60.0;
        
        // Temporarily boost target RPM when shooting
       // if (mCurrentRPM < mTargetRPM - 400) {
       //     mTargetRPS = mTargetRPS + 200; // Preemptive compensation
       // }

        double mCurrentRPS = mCurrentRPM / 60.0;
        
        double pidOutput = mShooterPID.calculate(mCurrentRPM, mTargetRPM);

        double ffOutput = tempFF.calculate(mTargetRPM); 
        //TODO: test with different feedforward 
       // double ffVelocityOutput = tempFF.calculateWithVelocities(mCurrentRPS,mTargetRPS); 

        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);

        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
        

    }

    public void runShooterPower(double motorPower){
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
    }
     // Finds the average velocity of the two motors 
    public double getVelocity() {
        double sum = mShooterLeaderEncoder.getVelocity() + mShooterFollowerEncoder.getVelocity();
        double average = sum / 2;
        return average;
    }

    public void stopAll() {
            runShooterPower(0);
            runKicker(0);
        }

    public void runKicker(double speed){
            mKicker.set(speed);
        }



    // --- Tuning Methods ---
    
    // Add tuning mode enum
    public enum TuningMode {
        KP, KV, KD, 
    }
    
    private TuningMode mCurrentTuningMode = TuningMode.KV;
    
    // Cycle through tuning modes
    public void cycleTuningMode() {
        switch (mCurrentTuningMode) {
            case KP:
                mCurrentTuningMode = TuningMode.KP;
                break;
            case KV:
                mCurrentTuningMode = TuningMode.KV;
                break;
            case KD:
                mCurrentTuningMode = TuningMode.KD;
                break;
        }
    }
    
    // Universal increment based on current mode
    public void incrementCurrentGain() {
        switch (mCurrentTuningMode) {
            case KP:
                incrementKP();
                break;
            case KV:
                incrementKV();
                break;
            case KD:
                incrementKD();
                break;
        }
    }
    
    // Universal decrement based on current mode
    public void decrementCurrentGain() {
        switch (mCurrentTuningMode) {
            case KP:
                decrementKP();
                break;
            case KV:
                decrementKV();
                break;
            case KD:
                decrementKD();
                break;
        }
    }
    public void incrementRPM() { mTargetRPM += ShooterConstants.RPM_INCREMENT; }
    public void decrementRPM() { mTargetRPM -= ShooterConstants.RPM_INCREMENT; }

    public void incrementKD() { mCurrentKD += ShooterConstants.KD_INCREMENT; } // Increments by 0.01 for fine tuning
    public void decrementKD() { mCurrentKD -= ShooterConstants.KD_INCREMENT; }

    public void incrementKI() { mCurrentKI += ShooterConstants.KI_INCREMENT; } // Increments by 0.01 for fine tuning
    public void decrementKI() { mCurrentKI -= ShooterConstants.KI_INCREMENT; }

    public void incrementKP() { mCurrentKP += ShooterConstants.KP_INCREMENT; } // Increments by 0.01 for fine tuning
    public void decrementKP() { mCurrentKP -= ShooterConstants.KP_INCREMENT; }

    public void incrementKV() { mCurrentKV += ShooterConstants.KV_INCREMENT; } // Increments by 0.01 for fine tuning
    public void decrementKV() { mCurrentKV -= ShooterConstants.KV_INCREMENT; }

    
    // --- Command Factories ---
    public Command runShooterCommand() { return run(this::runShooterPIDF); }
    //public Command runBangBangShooterCommand() { return run(this::runShooterBangBang); }

    
    public Command runShooterPowerCommand() {
         return run(
        () -> {
            runShooterPower(ShooterConstants.SHOOTER_SPEED);
              });
    }
    
   

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

    




    



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Testing/Target RPM", mTargetRPM);
        SmartDashboard.putNumber("Testing/Actual RPM", mShooterLeaderEncoder.getVelocity());
        SmartDashboard.putNumber("Testing/Current kD Tuning", mCurrentKD);
        SmartDashboard.putNumber("Testing/Current kP Tuning", mCurrentKP);
        SmartDashboard.putNumber("Testing/Current kI Tuning", mCurrentKI);
        SmartDashboard.putNumber("Testing/Current kV Tuning", mCurrentKV);
        SmartDashboard.putNumber("Testing/shooter current", mShooterLeader.getOutputCurrent());
        SmartDashboard.putNumber("Testing/shooter motor 2 current", mShooterFollower.getOutputCurrent());
        SmartDashboard.putString("Testing/Tuning Mode", mCurrentTuningMode.toString());



        
        boolean atSpeed = Math.abs(mShooterLeaderEncoder.getVelocity() - mTargetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
        SmartDashboard.putBoolean("Testing/Shooter Ready", atSpeed);
    }




    //DEPRECATED 

    /* public void runShooterBangBang() {
        //updatereaction time is not fast enough
        SimpleMotorFeedforward tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS, ShooterConstants.LEADER_FF_kV);

        double bbOutput = mShooterBangBang.calculate(mShooterEncoder.getVelocity(), mTargetRPM);
        double ffOutput = tempFF.calculate(mTargetRPM / 60.0); 
        
        mShooterLeader.set(MathUtil.clamp(bbOutput*60 + ffOutput, 0.0, 1.0));

    } */
}