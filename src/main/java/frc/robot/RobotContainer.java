package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TestingSubsystem;

public class RobotContainer {
    // --- Subsystems ---
    private final TestingSubsystem m_testingSubsystem = new TestingSubsystem();

    // --- Controller ---
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // Configure the button bindings
        configureBindings();
    }

    private void configureBindings() {
      
        // Y Button: Run Shooter
       m_driverController.y().toggleOnTrue(m_testingSubsystem.runShooterCommand());
        m_driverController.a().toggleOnTrue(m_testingSubsystem.runBangBangShooterCommand());

          //                   .toggleOnFalse(m_testingSubsystem.runOnce(m_testingSubsystem::stopAll));
       m_driverController.x().toggleOnFalse(m_testingSubsystem.runOnce(m_testingSubsystem::stopAll));

        // D-Pad Up/Down: Target RPM
        m_driverController.povUp().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::incrementKD));
        m_driverController.povDown().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::decrementKD));

        m_driverController.back().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::incrementKI));
        m_driverController.start().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::decrementKI));

        // D-Pad Left/Right: kV Tuning
        m_driverController.povRight().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::incrementKP));
        m_driverController.povLeft().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::decrementKP));

        // Bumpers: Kicker and Intake
        m_driverController.rightBumper().whileTrue(shoot());
        m_driverController.rightTrigger().whileTrue(m_testingSubsystem.runKickerCommand());
        m_driverController.leftBumper().whileTrue(m_testingSubsystem.runIntakeForwardCommand());
        m_driverController.leftTrigger().whileTrue(m_testingSubsystem.runIntakeBackwardCommand());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Returns a simple command to do nothing for now
        return Commands.none();
    }

    public Command shoot(){
    return Commands.parallel(           
    new RunCommand(() -> m_testingSubsystem.runIntake(Constants.IntakeConstants.INTAKE_SPEED)),
    new RunCommand(() -> m_testingSubsystem.runKicker(-Constants.ShooterConstants.KICKER_SPEED)));
  }

}
