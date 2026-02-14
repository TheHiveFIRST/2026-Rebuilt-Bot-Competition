package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.UnjamCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TestingSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;



import static edu.wpi.first.units.Units.RPM;


public class RobotContainer {
    // --- Subsystems ---
   private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
      private final TestingSubsystem mTestingSubsystem = new TestingSubsystem();

    private final ArmSubsystem mArmSubsystem = new ArmSubsystem();

    //private final FlywheelSubsystem mFlywheelSubsystem = new FlywheelSubsystem();

    // --- Controller ---
    // Only ONE controller instance to prevent HID conflicts
    private final CommandXboxController mDriverController = 
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();

        // Set the default command to force the shooter rest.
        //mFlywheelSubsystem.setDefaultCommand(mFlywheelSubsystem.set(0));
        mArmSubsystem.setDefaultCommand(mArmSubsystem.stopPivot());
        //mTestingSubsystem.setDefaultCommand(new RunCommand(()-> mTestingSubsystem.runShooterPower(0), mTestingSubsystem));
        mIntakeSubsystem.setDefaultCommand(new RunCommand(()-> mIntakeSubsystem.runIntake(0), mIntakeSubsystem));
        mTestingSubsystem.setDefaultCommand(new RunCommand(()-> mTestingSubsystem.runKicker(0), mTestingSubsystem));
     }

    private void configureBindings() {
        // --- ARM / STINGER (A Button Toggle) ---
        /* 
        mDriverController.a().onTrue(
            Commands.runOnce(() -> {
                // If current position is near the OUT setpoint, move to IN. Otherwise, move OUT.
                if (Math.abs(mArmSubsystem.encoderGetValue() - ArmConstants.PIVOT_OUT) < 0.05) {
                    mArmSubsystem.setTargetArm(ArmConstants.PIVOT_IN);
                } else {
                    mArmSubsystem.setTargetArm(ArmConstants.PIVOT_OUT);
                }
            }, mArmSubsystem)
        );
        */

        //INTAKE TUNING 
        /* 
        mDriverController.a().whileTrue(mArmSubsystem.runIntakePivotGround());
        mDriverController.b().whileTrue(mArmSubsystem.runIntakePivotUp());
        mDriverController.y().whileTrue(mArmSubsystem.runPivot());
        mDriverController.x().whileTrue(mArmSubsystem.runPivot12()); 
        */
        //mDriverController.povLeft().onTrue(mArmSubsystem.runOnce(mArmSubsystem::incrementKP));
       // mDriverController.povRight().onTrue(mArmSubsystem.runOnce(mArmSubsystem::decrementKP));
        




        // --- SHOOTER CONTROLS ---
       mDriverController.y().whileTrue(mTestingSubsystem.runShooterCommand());
       mDriverController.a().whileTrue(mTestingSubsystem.runShooterPowerCommand());

       mDriverController.x().toggleOnTrue(mTestingSubsystem.stop());


        // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // cancelling on release.
        //mDriverController.b().whileTrue(mFlywheelSubsystem.setVelocity(RPM.of(ShooterConstants.DEFAULT_TARGET_RPM)));
       // mDriverController.leftTrigger().whileTrue(mFlywheelSubsystem.set(0.3));



        // Bumpers: Kicker and Intake
        mDriverController.rightBumper().whileTrue(shoot());
        //mDriverController.rightTrigger().whileTrue(mTestingSubsystem.runKickerCommand());
       // mDriverController.leftBumper().whileTrue(mTestingSubsystem.runIntakeForwardCommand());
       // mDriverController.leftTrigger().whileTrue(mTestingSubsystem.runIntakeBackwardCommand());

        // ===== PID TUNING =====
        // Back Button (Double Tap): Cycle through KP -> KI -> KD tuning mode
       // mDriverController.back()
         //   .whileTrue(mTestingSubsystem.run(mTestingSubsystem::cycleTuningMode))
         //   .debounce(0.3); // Prevents accidental double presses
        

        // --- TUNING (D-Pad) ---
        mDriverController.povLeft().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::incrementRPM));
        mDriverController.povRight().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::decrementRPM));
        mDriverController.povUp().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::incrementCurrentGain));
        mDriverController.povDown().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::decrementCurrentGain));
        mDriverController.start().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::incrementKP));
        mDriverController.back().onTrue(mTestingSubsystem.runOnce(mTestingSubsystem::decrementKP));




        // --- INTAKE & KICKER ---
        mDriverController.rightTrigger().whileTrue(mTestingSubsystem.runKickerCommand());
        mDriverController.leftTrigger().whileTrue(mIntakeSubsystem.runIntakeForwardCommand());
        mDriverController.leftBumper().whileTrue(shootwithJam());

    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

   public Command shoot(){
    return Commands.parallel(           
    new RunCommand(() -> mIntakeSubsystem.runIntake(Constants.IntakeConstants.INTAKE_SPEED)),
    new RunCommand(() -> mTestingSubsystem.runKicker(-Constants.ShooterConstants.KICKER_SPEED)));
  }
  public Command shootwithJam(){
    return Commands.parallel(           
    new UnjamCommand(mIntakeSubsystem),
    mTestingSubsystem.runKickerCommand());
  }


}



