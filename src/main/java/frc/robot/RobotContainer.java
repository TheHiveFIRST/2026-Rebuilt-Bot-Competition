package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final TestingSubsystem mTestingSubsystem = new TestingSubsystem();

    // --- Controller ---
    private final CommandXboxController mDriverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // Configure the button bindings
        configureBindings();
    }

    private void configureBindings() {
      
        // Y Button: Run Shooter
        mDriverController.y().toggleOnTrue(mTestingSubsystem.runShooterCommand());
        mDriverController.a().toggleOnTrue(mTestingSubsystem.runShooterPowerCommand());

        mDriverController.x().toggleOnFalse(mTestingSubsystem.runOnce(mTestingSubsystem::stopAll));

        // Bumpers: Kicker and Intake
        mDriverController.rightBumper().whileTrue(shoot());
        mDriverController.rightTrigger().whileTrue(mTestingSubsystem.runKickerCommand());
        mDriverController.leftBumper().whileTrue(mTestingSubsystem.runIntakeForwardCommand());
        mDriverController.leftTrigger().whileTrue(mTestingSubsystem.runIntakeBackwardCommand());

                // ===== PID TUNING =====
        // Back Button (Double Tap): Cycle through KP -> KI -> KD tuning mode
        mDriverController.back()
            .onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::cycleTuningMode))
            .debounce(0.3); // Prevents accidental double presses
        
        // POV Up: Increment current gain (KP, KI, or KD depending on mode)
        mDriverController.povUp()
            .onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::incrementCurrentGain));
        
        // POV Down: Decrement current gain
        mDriverController.povDown()
            .onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::decrementCurrentGain));
        
        // POV Left: Decrease RPM target
        mDriverController.povLeft()
            .onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::decrementRPM));

        mDriverController.povRight()
            .onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::incrementRPM));

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
    new RunCommand(() -> mTestingSubsystem.runIntake(Constants.IntakeConstants.INTAKE_SPEED)),
    new RunCommand(() -> mTestingSubsystem.runKicker(-Constants.ShooterConstants.KICKER_SPEED)));
  }

}
