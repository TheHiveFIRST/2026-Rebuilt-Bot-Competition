package frc.robot.subsystems;

import com.fasterxml.jackson.databind.RuntimeJsonMappingException;
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

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs.ShooterConfig;
import frc.robot.Configs.IntakeConfigs;

public class TestingSubsystem extends SubsystemBase {

    private final SparkMax m_intakeLeader;
    private final SparkMax m_intakeFollower;
    private final SparkMax m_shooterLeader;
    private final SparkMax m_shooterFollower;
    private final SparkMax m_kicker;

    private final RelativeEncoder m_shooterEncoder;
    private final PIDController m_shooterPID;
    private final BangBangController m_shooterBangBang;

    
    // --- State Variables ---
    private double m_targetRPM = ShooterConstants.DEFAULT_TARGET_RPM;
    private double m_currentKV = ShooterConstants.LEADER_FF_kV;
    private double m_currentKP = ShooterConstants.LEADER_Kp;


    public TestingSubsystem() {
        m_intakeLeader = new SparkMax(IntakeConstants.INTAKE_LEADER_ID, MotorType.kBrushless);
        m_intakeFollower = new SparkMax(IntakeConstants.INTAKE_FOLLOWER_ID, MotorType.kBrushless);
        m_shooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
        m_shooterFollower = new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
        m_kicker = new SparkMax(ShooterConstants.SHOOTER_FEEDER, MotorType.kBrushless);

        m_intakeLeader.configure(IntakeConfigs.intakeLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeFollower.configure(IntakeConfigs.intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_kicker.configure(ShooterConfig.shooterFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_shooterEncoder = m_shooterLeader.getEncoder();
        m_shooterPID = new PIDController(m_currentKP, 0, 0);
    
        m_shooterBangBang = new BangBangController();
        m_shooterBangBang.setTolerance(30); //TODO: must test if tolerance needed/works for PID and bang bang
        
    }

    public void runShooterAtTarget() {
        // We create a temporary FF object using the live m_currentKV variable
        SimpleMotorFeedforward tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS, ShooterConstants.LEADER_FF_kV);

            
        m_shooterPID.setP(m_currentKP);
        double pidOutput = m_shooterPID.calculate(m_shooterEncoder.getVelocity(), m_targetRPM);
        double ffOutput = tempFF.calculate(m_targetRPM / 60.0); 
        
        m_shooterLeader.set(MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0));

        SmartDashboard.putNumber("current KP", m_currentKP);
    }

    public void runShooterBangBang() {
        // We create a temporary FF object using the live m_currentKV variable
        SimpleMotorFeedforward tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS, ShooterConstants.LEADER_FF_kV);

        double bbOutput = m_shooterBangBang.calculate(m_shooterEncoder.getVelocity(), m_targetRPM);
        double ffOutput = tempFF.calculate(m_targetRPM / 60.0); 
        
        m_shooterLeader.set(MathUtil.clamp(bbOutput + 0.9*ffOutput, 0.0, 1.0));

    }

    // --- Tuning Methods ---
    public void incrementRPM() { m_targetRPM += ShooterConstants.RPM_INCREMENT; }
    public void decrementRPM() { m_targetRPM -= ShooterConstants.RPM_INCREMENT; }

    public void incrementKV() { m_currentKV += ShooterConstants.KV_INCREMENT; } // Increments by 0.01 for fine tuning
    public void decrementKV() { m_currentKV -= ShooterConstants.KV_INCREMENT; }

    public void incrementKP() { m_currentKP += ShooterConstants.KP_INCREMENT; } // Increments by 0.01 for fine tuning
    public void decrementKP() { m_currentKP -= ShooterConstants.KP_INCREMENT; }

    public void stopAll() {
        m_intakeLeader.set(0);
        m_shooterLeader.set(0);
        m_kicker.set(0);
    }

    public void runIntake(double speed){
        m_intakeLeader.set(speed);
    }

    public void runKicker(double speed){
        m_kicker.set(speed);
    }



    // --- Command Factories ---
    public Command runShooterCommand() { return run(this::runShooterAtTarget); }
   
    public Command runIntakeForwardCommand() {
         return run(
        () -> {
            runIntake(IntakeConstants.INTAKE_SPEED);
              });
    }
    
    public Command runIntakeBackwardCommand() {
         return run(
        () -> {
            runIntake(-IntakeConstants.INTAKE_SPEED);
              });
    }

    public Command runKickerCommand () {
         return run(
        () -> {
            runKicker(ShooterConstants.KICKER_SPEED);
              });
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Testing/Target RPM", m_targetRPM);
        SmartDashboard.putNumber("Testing/Actual RPM", m_shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Testing/Current kV Tuning", m_currentKV);
        SmartDashboard.putNumber("Testing/Current kP Tuning", m_currentKP);
        SmartDashboard.putNumber("Testing/intake bus voltage", m_intakeLeader.getBusVoltage());
        
        boolean atSpeed = Math.abs(m_shooterEncoder.getVelocity() - m_targetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
        SmartDashboard.putBoolean("Testing/Shooter Ready", atSpeed);
    }
}