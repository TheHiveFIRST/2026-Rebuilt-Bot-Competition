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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.ShootConfig.ShooterConfig;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax mShooterLeader;
    private final SparkMax mShooterFollower;
    private final SparkMax mKicker;


    private final RelativeEncoder mShooterLeaderEncoder;
    private final RelativeEncoder mShooterFollowerEncoder; 
    private final PIDController mShooterPID;

    
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


    }

    public void runShooterPIDF() {
        // We create a temporary FF object using the live mCurrentKV variable
        SimpleMotorFeedforward tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS, mCurrentKV, mCurrentKA);

            
        mShooterPID.setP(mCurrentKP);
        mShooterPID.setI(mCurrentKI);
        mShooterPID.setD(mCurrentKD);
        
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        
        double mTargetRPS = mTargetRPM / 60.0;
        
        double mCurrentRPS = mCurrentRPM / 60.0;
        
        double pidOutput = mShooterPID.calculate(mCurrentRPM, mTargetRPM);

        double ffOutput = tempFF.calculate(mTargetRPM); 
        

        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);

        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);

        //TODO: test with different feedforward, boosting target RPM  
        // double ffVelocityOutput = tempFF.calculateWithVelocities(mCurrentRPS,mTargetRPS);
        // Temporarily boost target RPM when shooting
       // if (mCurrentRPM < mTargetRPM - 400) {
       //     mTargetRPS = mTargetRPS + 200; // Preemptive compensation
       // }
 

    }

    public void runShooterPower(double motorPower){
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
    }

    public double runShooterForDistance(double distance){
        double shooterRegressionRPM = (Math.pow(distance, 4) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_4)
        + (Math.pow(distance, 3) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_3)
        + (Math.pow(distance, 2) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_2)
        + (Math.pow(distance, 1) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_1)
        +(Constants.ShooterConstants.REGRESSION_COEFFICIENT_0);
        return shooterRegressionRPM; 
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



    // Tuning Methods 
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

    public void incrementKD() { mCurrentKD += ShooterConstants.KD_INCREMENT; } 
    public void decrementKD() { mCurrentKD -= ShooterConstants.KD_INCREMENT; }

    public void incrementKI() { mCurrentKI += ShooterConstants.KI_INCREMENT; } 
    public void decrementKI() { mCurrentKI -= ShooterConstants.KI_INCREMENT; }

    public void incrementKP() { mCurrentKP += ShooterConstants.KP_INCREMENT; } 
    public void decrementKP() { mCurrentKP -= ShooterConstants.KP_INCREMENT; }

    public void incrementKV() { mCurrentKV += ShooterConstants.KV_INCREMENT; } 
    public void decrementKV() { mCurrentKV -= ShooterConstants.KV_INCREMENT; }

    //Commands 
    public Command runShooterCommand() { return run(this::runShooterPIDF); }

    
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
        SmartDashboard.putNumber("Shooter/Target RPM", mTargetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", mShooterLeaderEncoder.getVelocity());
        SmartDashboard.putNumber("Testing/Current kD Tuning", mCurrentKD);
        SmartDashboard.putNumber("Testing/Current kP Tuning", mCurrentKP);
        SmartDashboard.putNumber("Testing/Current kI Tuning", mCurrentKI);
        SmartDashboard.putNumber("Testing/Current kV Tuning", mCurrentKV);
        SmartDashboard.putNumber("Testing/shooter current", mShooterLeader.getOutputCurrent());
        SmartDashboard.putNumber("Testing/shooter motor 2 current", mShooterFollower.getOutputCurrent());
        SmartDashboard.putString("Testing/Tuning Mode", mCurrentTuningMode.toString());

        boolean atSpeed = Math.abs(mShooterLeaderEncoder.getVelocity() - mTargetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
        SmartDashboard.putBoolean("Shooter/Shooter Ready", atSpeed);
    }

}