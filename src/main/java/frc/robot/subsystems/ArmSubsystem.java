package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax mPivotLeader;
    private final SparkMax mPivotFollower;
    private final AbsoluteEncoder mPivotEncoder;
    private final PIDController mPivotPID;
    public double mPivotOutput = 0; 

    private double mCurrentTarget = ArmConstants.PIVOT_IN;

    public ArmSubsystem() {
        mPivotLeader = new SparkMax(ArmConstants.ARM_LEADER_ID, MotorType.kBrushless);
        mPivotFollower = new SparkMax(ArmConstants.ARM_FOLLOWER_ID, MotorType.kBrushless);
        
        // Configuration for Leader
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake);
        // If the arm moves the wrong way, uncomment the line below:
        // leaderConfig.inverted(true); 

        // Configuration for Follower
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(ArmConstants.ARM_LEADER_ID);
        followerConfig.idleMode(IdleMode.kBrake);

        // Applying configurations using 2026 REV syntax
        mPivotLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mPivotFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mPivotEncoder = mPivotLeader.getAbsoluteEncoder();
        mPivotPID = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
        mPivotPID.setTolerance(ArmConstants.POSITION_TOLERANCE);
    }

    public void setTargetArm(double position) {
        mCurrentTarget = position;
        double mPivotOutput = mPivotPID.calculate(mPivotEncoder.getPosition(), mCurrentTarget);
        mPivotLeader.set(mPivotOutput);
    }

    public double encoderGetValue() {
        return mPivotEncoder.getPosition();
    }

    public boolean isAtTarget() {
        return mPivotPID.atSetpoint();
    }

    public void stopArm() {
            mPivotLeader.set(0);
            mCurrentTarget = mPivotEncoder.getPosition(); // Hold current spot
        }

    
     public Command runIntakePivotCommand() {
         return run(
        () -> {
            setTargetArm(ArmConstants.PIVOT_IN);
              });
    }
    @Override
    public void periodic() {
    

        // Debugging values to Glass/SmartDashboard
        SmartDashboard.putNumber("Arm/Encoder Value", encoderGetValue());
        SmartDashboard.putNumber("Arm/Target Position", mCurrentTarget);
        SmartDashboard.putBoolean("Arm/At Target", isAtTarget());
    }

    
}