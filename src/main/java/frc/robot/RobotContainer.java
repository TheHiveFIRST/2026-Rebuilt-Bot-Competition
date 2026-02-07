package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.TestingSubsystem;

public class RobotContainer {
    private final CommandXboxController m_driverController = 
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final TestingSubsystem m_testingSubsystem = new TestingSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Y Button: Runs Shooter at the target RPM defined in the subsystem
        m_driverController.y().whileTrue(m_testingSubsystem.runShooterCommand())
                             .onFalse(m_testingSubsystem.runOnce(m_testingSubsystem::stopAll));

        // D-Pad Up/Down: Increments/Decrements target RPM by 10
        m_driverController.povUp().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::incrementRPM));
        m_driverController.povDown().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::decrementRPM));

        // Right Bumper: Run Kicker (Feeder)
        m_driverController.rightBumper().whileTrue(m_testingSubsystem.runKickerCommand(0.5));

        // Left Bumper: Run Intake
        m_driverController.leftBumper().whileTrue(m_testingSubsystem.runIntakeCommand());
    }
}