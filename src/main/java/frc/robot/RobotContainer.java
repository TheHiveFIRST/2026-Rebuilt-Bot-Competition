package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TestingSubsystem;

public class RobotContainer {
    // --- Subsystems ---
    private final TestingSubsystem m_testingSubsystem = new TestingSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();

    // --- Controller ---
    // Only ONE controller instance to prevent HID conflicts
    private final CommandXboxController m_driverController = 
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // --- ARM / STINGER (A Button Toggle) ---
        m_driverController.a().onTrue(
            Commands.runOnce(() -> {
                // If current position is near the OUT setpoint, move to IN. Otherwise, move OUT.
                if (Math.abs(m_arm.encoderGetValue() - ArmConstants.kPivotOut) < 0.05) {
                    m_arm.setTargetPosition(ArmConstants.kPivotIn);
                } else {
                    m_arm.setTargetPosition(ArmConstants.kPivotOut);
                }
            }, m_arm)
        );

        // --- SHOOTER CONTROLS ---
        m_driverController.y().toggleOnTrue(m_testingSubsystem.runShooterCommand());
        
        // X Button as a "Stop Everything" safety
        m_driverController.x().onTrue(Commands.runOnce(() -> {
            m_testingSubsystem.stopAll();
            // Optional: You can also force the arm to stop moving here
        }, m_testingSubsystem, m_arm));

        // --- TUNING (D-Pad) ---
        m_driverController.povUp().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::incrementRPM));
        m_driverController.povDown().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::decrementRPM));
        m_driverController.povRight().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::incrementKP));
        m_driverController.povLeft().onTrue(m_testingSubsystem.runOnce(m_testingSubsystem::decrementKP));

        // --- INTAKE & KICKER ---
        m_driverController.rightBumper().whileTrue(m_testingSubsystem.runKickerCommand());
        m_driverController.leftBumper().whileTrue(m_testingSubsystem.runIntakeForwardCommand());
        m_driverController.leftTrigger().whileTrue(m_testingSubsystem.runIntakeBackwardCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}