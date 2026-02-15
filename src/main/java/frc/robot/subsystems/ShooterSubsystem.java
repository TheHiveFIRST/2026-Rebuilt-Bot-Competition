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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.configs.ShootConfig;

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


        mShooterLeader.configure(ShootConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFollower.configure(ShootConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mKicker.configure(ShootConfig.shooterFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    public void runShooterForDistance(double distancetohub){
        double shooterregresiontarget = (Math.pow(distancetohub, 4) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_4)
        + (Math.pow(distancetohub, 3) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_3)
        + (Math.pow(distancetohub, 2) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_2)
        + (Math.pow(distancetohub, 1) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_1)
        +(Constants.ShooterConstants.REGRESSION_COEFFICIENT_0);
        runShooterPower(shooterregresiontarget); //TODO: FIX METHOD TO OUTPUT SHOOTER POWER

    }
   
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