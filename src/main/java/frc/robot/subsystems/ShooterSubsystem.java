package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Configs.ShooterConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//constructor 
public class ShooterSubsystem extends SubsystemBase {

    private SparkMax mShooterLeader;
    private SparkMax mShooterFollower;
    private SparkMax mShooterFeeder;

    // Encoders and PIDF controllers
    private RelativeEncoder leaderEncoder;

    private PIDController leaderPID;
    private SimpleMotorFeedforward leaderFF;

    public ShooterSubsystem() {
        // Initialize motors
        mShooterLeader = new SparkMax(Constants.ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
        mShooterFollower = new SparkMax(Constants.ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
        mShooterFeeder = new SparkMax(Constants.ShooterConstants.SHOOTER_FEEDER, MotorType.kBrushless);

        // Configure motors
        mShooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFeeder.configure(ShooterConfig.shooterFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize encoders and PID controllers

        leaderEncoder = mShooterLeader.getEncoder();
        leaderPID = new PIDController(Constants.ShooterConstants.LEADER_Kp, Constants.ShooterConstants.LEADER_Ki, Constants.ShooterConstants.LEADER_Kd);

        // Initialize feedforward (assign to class-level fields)
        leaderFF = new SimpleMotorFeedforward(Constants.ShooterConstants.LEADER_FF_kS, Constants.ShooterConstants.LEADER_FF_kV, 0);
    }

//methods 
public void runShooterVelocity(double targetRPM) {
    // Calculate PID for speed
    double LeaderPIDOutput = leaderPID.calculate(leaderEncoder.getVelocity(), targetRPM);
  

    // Calculate feedforward
    double targetRPS = targetRPM / 60.0;
    double leaderFFOutput = leaderFF.calculate(targetRPS);


    double leaderspeed = MathUtil.clamp(LeaderPIDOutput + leaderFFOutput, -1.0, 1.0);


    SmartDashboard.putNumber("Leader Shooter RPM", leaderEncoder.getVelocity());
    runShooter(leaderspeed);
}

public void runShooter(double leaderspeed) {
    // mShooterLeader.set(leaderspeed);
    System.out.println(leaderspeed);
}

// for the feeder shooter
public void feedShooter(double speed) {
    mShooterFeeder.set(speed);
}

public void runShooterForDistance(double distancetohub){
    double shooterregresiontarget = (Math.pow(distancetohub, 4) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_4)
    + (Math.pow(distancetohub, 3) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_3)
    + (Math.pow(distancetohub, 2) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_2)
    + (Math.pow(distancetohub, 1) * Constants.ShooterConstants.REGRESSION_COEFFICIENT_1)
    +(Constants.ShooterConstants.REGRESSION_COEFFICIENT_0);
    runShooterVelocity(shooterregresiontarget);

}

//command to run shooter forwards
public Command runShooterForwards() 
{
    return run(
    () -> {
        runShooterVelocity(Constants.ShooterConstants.SHOOT_FORWARDS);
    });
}

public Command stopShooter()
{
    return run(
    () -> {
        runShooterVelocity(0);
    });
}


//command to feed fuel into the shooter
public Command feedShooterForwards(){
    return run(
    () -> {
        feedShooter(Constants.ShooterConstants.FEEDER_FEED);
    });
}

//command to run feeder backwards to unjam it or smth
public Command evacuateShooter(){
    return run(
    () -> {
        feedShooter(Constants.ShooterConstants.FEEDER_EVACUATE);
    });
}
public Command runshooterwithregression(){
    return run(
    () -> {
        runShooterForDistance(0);// the 0 is a placeholder. the real value needs to come from the limelight stuff
    });
}

}
