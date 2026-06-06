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
   
    public double mPivotOutputPower = 0; 
    private double mArmCurrentKP = ArmConstants.ARM_KP;
    private double mCurrentTarget = ArmConstants.PIVOT_IN;
    private boolean mIntakeGroundSecondStage = false; 

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

        //PID constants for arm 
        mPivotFollowerPID = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
        mPivotLeaderPID = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI,ArmConstants.ARM_KD);
        mPivotLeaderPID.setTolerance(ArmConstants.POSITION_TOLERANCE);
        mPivotFollowerPID.setTolerance(ArmConstants.POSITION_TOLERANCE);
    }

    // Methods

    /**
     * Set the desired pivot position and drive the motors with the follower PID.
     *
     * @param position desired pivot position (encoder units)
     */
    public void setTargetArmPosition(double position) {
        mCurrentTarget = position;
        // Use the field mPivotOutputPower (avoid shadowing a local variable)
        mPivotOutputPower = mPivotFollowerPID.calculate(mPivotFollowerEncoder.getPosition(), mCurrentTarget);
        mPivotFollower.set(mPivotOutputPower);
        mPivotLeader.set(mPivotOutputPower);
    }

    /**
     * Set arm pivot motor power directly (open-loop).
     *
     * @param pivotPower motor power in range [-1.0, 1.0]
     */
    public void setArmPower(double pivotPower) {
        mPivotLeader.set(pivotPower);
        mPivotFollower.set(pivotPower);
    }

    /**
     * Get the follower absolute encoder position.
     *
     * @return encoder position
     */
    public double getEncoderValue() {
        return mPivotFollowerEncoder.getPosition();
    }

    /**
     * Query whether the leader PID controller is at its setpoint.
     *
     * @return true if at setpoint
     */
    public boolean isAtTarget() {
        return mPivotLeaderPID.atSetpoint();
    }

    /**
     * Stop the arm motors and hold current position by updating the target.
     */
    public void stopArm() {
        mPivotLeader.set(0);
        mCurrentTarget = mPivotLeaderEncoder.getPosition(); // Hold current spot
    }

    /** Increase the internal KP tuning value by the configured increment. */
    public void incrementKP() {
        mArmCurrentKP += ArmConstants.ARM_KP_INCREMENT;
    }

    /** Decrease the internal KP tuning value by the configured increment. */
    public void decrementKP() {
        mArmCurrentKP -= ArmConstants.ARM_KP_INCREMENT;
    }

    

    //Commands 
    public Command IntakeToPositionCommand(double position) {
         return run(
        () -> {
            setTargetArmPosition(position);
              });
    }
    
    public Command runIntakePivotGroundCommand() {
         return run(
        () -> {
            if (getEncoderValue() < 0.7){
             setTargetArmPosition(ArmConstants.PIVOT_OUT);
            } else {
                setArmPower(0);
            }
              });
    }
    
    
    public Command runIntakePivotBumpCommand() {
         return run(
        () -> {
            setTargetArmPosition(ArmConstants.PIVOT_BUMP);
              });
    }
    
    public Command runIntakePivotInCommand() {
         return run(
        () -> {
            setTargetArmPosition(ArmConstants.PIVOT_IN);
              });
    }
    
    public Command IntakeSlightlyUpCommand() {
         return run(
        () -> {
            setTargetArmPosition(ArmConstants.PIVOT_AGITATE);
              });
    }
   

    public Command stopPivotCommand() {
         return run(
        () -> {
            setArmPower(0.0);
              });
    }

    public Command runPivotCommand(){
         return run(
        () -> {
            setArmPower(-0.2);
              });
    }

    public Command toggleArmStageCommand() {
        return new InstantCommand(() -> mIntakeGroundSecondStage = !mIntakeGroundSecondStage);}


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Pivot Encoder Value", getEncoderValue());
        SmartDashboard.putNumber("Arm/Target Position", mCurrentTarget);
    }

    
}