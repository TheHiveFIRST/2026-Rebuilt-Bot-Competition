package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs.ShooterConfig; // Assuming ShooterConfig handles the 14/15/16 setup
import frc.robot.Configs.IntakeConfigs;  // Assuming IntakeConfigs handles the 10/11 setup

public class TestingSubsystem extends SubsystemBase {
    // Hardware
    private final SparkMax m_intakeLeader = new SparkMax(IntakeConstants.INTAKE_LEADER_ID, MotorType.kBrushless);
    private final SparkMax m_intakeFollower = new SparkMax(IntakeConstants.INTAKE_FOLLOWER_ID, MotorType.kBrushless);
    private final SparkMax m_shooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
    private final SparkMax m_shooterFollower = new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
    private final SparkMax m_kicker = new SparkMax(ShooterConstants.KICKER_CANID, MotorType.kBrushless);

    // Control Objects
    private final RelativeEncoder m_shooterEncoder;
    private final PIDController m_shooterPID;
    private final SimpleMotorFeedforward m_shooterFF;

    // State Variables
    private double m_targetRPM = ShooterConstants.DEFAULT_TARGET_RPM;

    public TestingSubsystem() {
        // Configure Intake
        m_intakeLeader.configure(IntakeConfigs.intakeLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeFollower.configure(IntakeConfigs.intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Configure Shooter/Kicker
        m_shooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_kicker.configure(ShooterConfig.shooterFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_shooterEncoder = m_shooterLeader.getEncoder();
        m_shooterPID = new PIDController(ShooterConstants.LEADER_Kp, 0, 0);
        m_shooterFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS, ShooterConstants.LEADER_FF_kV);
    }

    // --- Action Methods ---

    public void runShooterAtTarget() {
        double pidOutput = m_shooterPID.calculate(m_shooterEncoder.getVelocity(), m_targetRPM);
        double ffOutput = m_shooterFF.calculate(m_targetRPM / 60.0);
        m_shooterLeader.set(MathUtil.clamp(pidOutput + ffOutput, -1.0, 1.0));
    }

    public void setIntake(double speed) { m_intakeLeader.set(speed); }
    public void setKicker(double speed) { m_kicker.set(speed); }
    
    public void incrementRPM() { m_targetRPM += ShooterConstants.RPM_INCREMENT; }
    public void decrementRPM() { m_targetRPM -= ShooterConstants.RPM_INCREMENT; }

    public void stopAll() {
        m_intakeLeader.set(0);
        m_shooterLeader.set(0);
        m_kicker.set(0);
    }

    // --- Command Factories ---

    public Command runShooterCommand() {
        return run(this::runShooterAtTarget);
    }

    public Command runIntakeCommand() {
        return startEnd(() -> setIntake(IntakeConstants.INTAKE_SPEED), () -> setIntake(0));
    }

    public Command runKickerCommand(double speed) {
        return startEnd(() -> setKicker(speed), () -> setKicker(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Testing/Target RPM", m_targetRPM);
        SmartDashboard.putNumber("Testing/Actual RPM", m_shooterEncoder.getVelocity());
    }
}