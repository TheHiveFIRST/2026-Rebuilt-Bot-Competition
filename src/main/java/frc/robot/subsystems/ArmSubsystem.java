package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax mPivotLeader;
    private final SparkMax mPivotFollower;
    private final AbsoluteEncoder mPivotFollowerEncoder;
    private final AbsoluteEncoder mPivotLeaderEncoder;
    private final PIDController mPivotLeaderPID;
    private final PIDController mPivotFollowerPID;
    public double mPivotOutput = 0; 

    //tuning 
    private double mArmCurrentKP = ArmConstants.ARM_KP;
    private double mArmCurrentKI = ArmConstants.ARM_KI;
    private double mArmCurrentKD = ArmConstants.ARM_KD;


    private double mCurrentTarget = ArmConstants.PIVOT_IN;

    private boolean intakeGroundSecondStage = false; 

    public ArmSubsystem() {
        mPivotLeader = new SparkMax(ArmConstants.ARM_LEADER_ID, MotorType.kBrushless);
        mPivotFollower = new SparkMax(ArmConstants.ARM_FOLLOWER_ID, MotorType.kBrushless);
        
        // Configuration for Leader
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(50);;

        // Configuration for Follower
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(50);;

        // Applying configuration`s using 2026 REV syntax
        mPivotLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mPivotFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mPivotFollowerEncoder = mPivotFollower.getAbsoluteEncoder();
        mPivotLeaderEncoder = mPivotLeader.getAbsoluteEncoder();
        mPivotFollowerPID = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
        mPivotLeaderPID = new PIDController(mArmCurrentKP, mArmCurrentKI, mArmCurrentKD);
        mPivotLeaderPID.setTolerance(ArmConstants.POSITION_TOLERANCE);
        mPivotFollowerPID.setTolerance(ArmConstants.POSITION_TOLERANCE);
    }

    public void setTargetArm(double position){
        mCurrentTarget = position; 
        double mPivotOutput = mPivotFollowerPID.calculate(mPivotFollowerEncoder.getPosition(), mCurrentTarget);
        mPivotFollower.set(mPivotOutput);
        mPivotLeader.set(mPivotOutput);
    }
    // public void set1stArm(double position) {
    //     mCurrentTarget = position;
    //     // mPivotPID.setP(mArmCurrentKP);
    //     // mPivotPID.setI(mArmCurrentKI);
    //     // mPivotPID.setD(mArmCurrentKD);

    //     double mPivotOutput = mPivotFollowerPID.calculate(mPivotFollowerEncoder.getPosition(), mCurrentTarget);
    //     mPivotFollower.set(mPivotOutput);
        
    // }

    // public void set2ndArm(double position){
    //     mPivotLeaderPID.setP(mArmCurrentKP);
    //     double mPivotLeaderOutput = mPivotLeaderPID.calculate(mPivotLeaderEncoder.getPosition(), mCurrentTarget);
    //     mPivotLeader.set(mPivotLeaderOutput);
    // }
    
    public void setPivotPower(double pivotPower){
        mPivotLeader.set(pivotPower);
        mPivotFollower.set(pivotPower);

    }


    public double encoderGetValue() {
        return mPivotLeaderEncoder.getPosition();
    }

    public boolean isAtTarget() {
        return mPivotLeaderPID.atSetpoint();
    }

    public void stopArm() {
            mPivotLeader.set(0);
            mCurrentTarget = mPivotLeaderEncoder.getPosition(); // Hold current spot
        }

    public void incrementKP() { mArmCurrentKP += ArmConstants.ARM_KP_INCREMENT; } // Increments by 0.01 for fine tuning
    public void decrementKP() { mArmCurrentKP -= ArmConstants.ARM_KP_INCREMENT; }

    
     public Command runIntakePivotGround () {
         return run(
        () -> {
            if (encoderGetValue() > 0.55 ){
             setPivotPower(0.0);
            }
            else {
             setTargetArm(ArmConstants.PIVOT_OUT);
            }
              });
    }

    public Command IntakePivotDefault() {
         return run(
        () -> {
            setTargetArm(ArmConstants.PIVOT_DEFAULT);
              });
    }

    public Command runIntakePivotUp() {
         return run(
        () -> {
            setTargetArm(ArmConstants.PIVOT_IN);
              });
    }

    public Command stopPivot() {
         return run(
        () -> {
            setPivotPower(0.0);
              });
    }

    public Command runPivot(){
         return run(
        () -> {
            setPivotPower(0.1);
              });
    }

    public Command toggleArmStageCommand() {
        return new InstantCommand(() -> intakeGroundSecondStage = !intakeGroundSecondStage);}




    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Encoder Value", encoderGetValue());
        SmartDashboard.putNumber("Arm/Target Position", mCurrentTarget);
        SmartDashboard.putBoolean("Arm/At Target", isAtTarget());
        SmartDashboard.putNumber("Arm/pivot current", mPivotLeader.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Current kP Tuning", mArmCurrentKP);
    }

    
}