package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.configs.ShooterConfig;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//constructor 
public class ShooterSubsystem extends SubsystemBase {

    private SparkMax mShooterRightLeader;
    private SparkMax mShooterRightFollower;
    private SparkMax mShooterLeftLeader;
    private SparkMax mShooterLeftFollower;

    // Encoders and PIDF controllers
    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;
    private PIDController rightPID;
    private PIDController leftPID;
    private SimpleMotorFeedforward RightFF;
    private SimpleMotorFeedforward LeftFF;

    public ShooterSubsystem() {
        // Initialize motors
        mShooterRightLeader = new SparkMax(Constants.ShooterConstants.SHOOTER_RIGHT_LEADER_CANID, MotorType.kBrushless);
        mShooterLeftLeader = new SparkMax(Constants.ShooterConstants.SHOOTER_LEFT_LEADER_CANID, MotorType.kBrushless);
        mShooterRightFollower = new SparkMax(Constants.ShooterConstants.SHOOTER_RIGHT_FOLLOWER_CANID, MotorType.kBrushless);
        mShooterLeftFollower = new SparkMax(Constants.ShooterConstants.SHOOTER_LEFT_FOLLOWER_CANID, MotorType.kBrushless);

        // Configure motors
        mShooterRightLeader.configure(ShooterConfig.shooterRightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterRightFollower.configure(ShooterConfig.shooterRightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterLeftLeader.configure(ShooterConfig.shooterLeftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterLeftFollower.configure(ShooterConfig.shooterLeftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize encoders and PID controllers
        rightEncoder = mShooterRightLeader.getEncoder();
        leftEncoder = mShooterLeftLeader.getEncoder();
        rightPID = new PIDController(Constants.ShooterConstants.RIGHT_Kp, Constants.ShooterConstants.RIGHT_Ki, Constants.ShooterConstants.RIGHT_Kd);
        leftPID = new PIDController(Constants.ShooterConstants.LEFT_Kp, Constants.ShooterConstants.LEFT_Ki, Constants.ShooterConstants.LEFT_Kd);

        // Initialize feedforward (assign to class-level fields)
        RightFF = new SimpleMotorFeedforward(Constants.ShooterConstants.RIGHT_FF_kS, Constants.ShooterConstants.RIGHT_FF_kV, 0);
        LeftFF = new SimpleMotorFeedforward(Constants.ShooterConstants.LEFT_FF_kS, Constants.ShooterConstants.LEFT_FF_kV, 0);
    }

//methods 
public void runShooterVelocity(double targetRPM) {
    // Calculate PID for speed
    double RightPIDOutput = rightPID.calculate(rightEncoder.getVelocity(), targetRPM);
    double LeftPIDOutput = leftPID.calculate(leftEncoder.getVelocity(), targetRPM);

    // Calculate feedforward
    double targetRPS = targetRPM / 60.0;
    double RightFFOutput = RightFF.calculate(targetRPS);
    double LeftFFOutput = LeftFF.calculate(targetRPS);

    double rspeed = MathUtil.clamp(RightPIDOutput + RightFFOutput, -1.0, 1.0);
    double lspeed = MathUtil.clamp(LeftPIDOutput + LeftFFOutput, -1.0, 1.0);


    SmartDashboard.putNumber("Right Shooter RPM", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Shooter RPM", leftEncoder.getVelocity());
    runShooter(rspeed, lspeed);
}

public void runShooter(double rspeed, double lspeed) {
    mShooterRightLeader.set(rspeed);
    mShooterLeftLeader.set(-lspeed);
}

//command to run shooter forwards
public Command runShooterForwards() 
{
    return run(
    () -> {
        runShooterVelocity(Constants.ShooterConstants.SHOOT_FORWARDS);
    });
}

//command to run shooter backwards

public Command runShooterBackwards()
{
    return run(
    () -> {
        runShooterVelocity(Constants.ShooterConstants.SHOOT_BACKWARDS);
    });
}
}
