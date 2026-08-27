package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.configs.ShootConfig.IntakeConfigs;


public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex mIntakeMotor;
    


    public IntakeSubsystem() {
        mIntakeMotor = new SparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        
        mIntakeMotor.configure(IntakeConfigs.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void stopIntake() {
            runIntake(0);
        }

    public void runIntake(double speed){
            mIntakeMotor.set(speed);
        }


    public Command runIntakeForwardCommand() {
         return run(
        () -> {
            runIntake(IntakeConstants.INTAKE_SPEED);
              });
    }

    public Command runIntakeSlowCommand() {
         return run(
        () -> {
            runIntake(IntakeConstants.SLOW_INTAKE_SPEED);
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
        //SmartDashboard.putNumber("Intake/intake bus voltage", mIntakeLeader.getBusVoltage());


    }



}