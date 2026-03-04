package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.configs.ShootConfig.IntakeConfigs;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax mIntakeLeader;
    private final SparkMax mIntakeFollower;


    public IntakeSubsystem() {
        mIntakeLeader = new SparkMax(IntakeConstants.INTAKE_LEADER_ID, MotorType.kBrushless);
        mIntakeFollower = new SparkMax(IntakeConstants.INTAKE_FOLLOWER_ID, MotorType.kBrushless);

        mIntakeLeader.configure(IntakeConfigs.intakeLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mIntakeFollower.configure(IntakeConfigs.intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void stopIntake() {
            runIntake(0);
        }

    public void runIntake(double speed){
            mIntakeLeader.set(speed);
        }


    public Command runIntakeForwardCommand() {
         return run(
        () -> {
            runIntake(IntakeConstants.INTAKE_SPEED);
              });
    }
    
    public Command runOuttakeCommand() {
         return run(
        () -> {
            runIntake(IntakeConstants.OUTTAKE_SPEED);
              });
    }


    public Command stop() {
         return run(
        () -> {
            stopIntake();
              });
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/intake bus voltage", mIntakeLeader.getBusVoltage());


    }



}