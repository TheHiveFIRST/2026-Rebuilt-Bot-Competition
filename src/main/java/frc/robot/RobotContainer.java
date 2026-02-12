package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TestingSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;


import static edu.wpi.first.units.Units.RPM;


public class RobotContainer {
    // --- Subsystems ---
    private final TestingSubsystem mTestingSubsystem = new TestingSubsystem();
    private final ArmSubsystem mArm = new ArmSubsystem();

    private final FlywheelSubsystem mFlywheelSubsystem = new FlywheelSubsystem();

    // --- Controller ---
    // Only ONE controller instance to prevent HID conflicts
    private final CommandXboxController mDriverController = 
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();

        // Set the default command to force the shooter rest.
        mFlywheelSubsystem.setDefaultCommand(mFlywheelSubsystem.set(0));
     }

    private void configureBindings() {
        // --- ARM / STINGER (A Button Toggle) ---
        mDriverController.a().onTrue(
            Commands.runOnce(() -> {
                // If current position is near the OUT setpoint, move to IN. Otherwise, move OUT.
                if (Math.abs(mArm.encoderGetValue() - ArmConstants.PIVOT_OUT) < 0.05) {
                    mArm.setTargetPosition(ArmConstants.PIVOT_IN);
                } else {
                    mArm.setTargetPosition(ArmConstants.PIVOT_OUT);
                }
            }, mArm)
        );

        // --- SHOOTER CONTROLS ---
        mDriverController.y().toggleOnTrue(mTestingSubsystem.runShooterCommand());
       // mDriverController.x().toggleOnFalse(mTestingSubsystem.runOnce(mTestingSubsystem::stopAll));


        // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // cancelling on release.
        mDriverController.b().whileTrue(mFlywheelSubsystem.setVelocity(RPM.of(ShooterConstants.DEFAULT_TARGET_RPM)));
        mDriverController.leftTrigger().whileTrue(mFlywheelSubsystem.set(0.3));



        // Bumpers: Kicker and Intake
        mDriverController.rightBumper().whileTrue(shoot());
        mDriverController.rightTrigger().whileTrue(mTestingSubsystem.runKickerCommand());
        mDriverController.leftBumper().whileTrue(mTestingSubsystem.runIntakeForwardCommand());
       // mDriverController.leftTrigger().whileTrue(mTestingSubsystem.runIntakeBackwardCommand());

        // ===== PID TUNING =====
        // Back Button (Double Tap): Cycle through KP -> KI -> KD tuning mode
        mDriverController.back()
            .onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::cycleTuningMode))
            .debounce(0.3); // Prevents accidental double presses
        
        //
        mDriverController.x().onTrue(Commands.runOnce(() -> {
            mTestingSubsystem.stopAll();
        }, mTestingSubsystem));

        // --- TUNING (D-Pad) ---
        mDriverController.povLeft().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::incrementRPM));
        mDriverController.povRight().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::decrementRPM));
        mDriverController.povUp().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::incrementCurrentGain));
        mDriverController.povDown().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::decrementCurrentGain));



        // --- INTAKE & KICKER ---
        mDriverController.rightBumper().whileTrue(mTestingSubsystem.runKickerCommand());
        mDriverController.leftBumper().whileTrue(mTestingSubsystem.runIntakeForwardCommand());
        mDriverController.leftTrigger().whileTrue(mTestingSubsystem.runIntakeBackwardCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public Command shoot(){
    return Commands.parallel(           
    new RunCommand(() -> mTestingSubsystem.runIntake(Constants.IntakeConstants.INTAKE_SPEED)),
    new RunCommand(() -> mTestingSubsystem.runKicker(-Constants.ShooterConstants.KICKER_SPEED)));
  }

}



