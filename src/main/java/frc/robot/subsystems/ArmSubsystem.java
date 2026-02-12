package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax m_leader;
    private final SparkMax m_follower;
    private final AbsoluteEncoder m_encoder;
    private final PIDController m_pid;

    private double m_currentTarget = ArmConstants.kPivotIn;

    public ArmSubsystem() {
        m_leader = new SparkMax(ArmConstants.kArmLeaderId, MotorType.kBrushless);
        m_follower = new SparkMax(ArmConstants.kArmFollowerId, MotorType.kBrushless);
        
        // Configuration for Leader
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake);
        // If the arm moves the wrong way, uncomment the line below:
        // leaderConfig.inverted(true); 

        // Configuration for Follower
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(ArmConstants.kArmLeaderId);
        followerConfig.idleMode(IdleMode.kBrake);

        // Applying configurations using 2026 REV syntax
        m_leader.configure(leaderConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        m_encoder = m_leader.getAbsoluteEncoder();
        m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        m_pid.setTolerance(ArmConstants.kPositionTolerance);
    }

    public void setTargetPosition(double position) {
        m_currentTarget = position;
    }

    public double encoderGetValue() {
        return m_encoder.getPosition();
    }

    public boolean isAtTarget() {
        return m_pid.atSetpoint();
    }

    @Override
    public void periodic() {
        // Automatically move to the current target position
        double pidOutput = m_pid.calculate(m_encoder.getPosition(), m_currentTarget);
        m_leader.set(pidOutput);

        // Debugging values to Glass/SmartDashboard
        SmartDashboard.putNumber("Arm/Encoder Value", encoderGetValue());
        SmartDashboard.putNumber("Arm/Target Position", m_currentTarget);
        SmartDashboard.putBoolean("Arm/At Target", isAtTarget());
    }

    public void stopArm() {
        m_leader.set(0);
        m_currentTarget = m_encoder.getPosition(); // Hold current spot
    }
}