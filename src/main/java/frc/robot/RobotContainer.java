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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;





public class RobotContainer {
    // --- Subsystems ---
   private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
      private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();

    private final ArmSubsystem mArmSubsystem = new ArmSubsystem();

    //private final FlywheelSubsystem mFlywheelSubsystem = new FlywheelSubsystem();

    // --- Controller ---
    // Only ONE controller instance to prevent HID conflicts
    private final CommandXboxController mDriverController = 
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController mOperatorController = 
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    public RobotContainer() {
        configureBindings();

        // Set the default command to force the shooter rest.
        //mFlywheelSubsystem.setDefaultCommand(mFlywheelSubsystem.set(0));
        mArmSubsystem.setDefaultCommand(mArmSubsystem.stopPivot());
        //mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runShooterPower(0), mShooterSubsystem));
        mIntakeSubsystem.setDefaultCommand(new RunCommand(()-> mIntakeSubsystem.runIntake(0), mIntakeSubsystem));
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runKicker(0), mShooterSubsystem));
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
        
        mOperatorController.a().whileTrue(mArmSubsystem.runIntakePivotGround());
        mOperatorController.b().whileTrue(mArmSubsystem.runIntakePivotUp());
        mOperatorController.y().whileTrue(mArmSubsystem.runPivot());
        
        mOperatorController.povLeft().onTrue(mArmSubsystem.runOnce(mArmSubsystem::incrementKP));
        mOperatorController.povRight().onTrue(mArmSubsystem.runOnce(mArmSubsystem::decrementKP));
        




        // --- SHOOTER CONTROLS ---
       mDriverController.y().whileTrue(mShooterSubsystem.runShooterCommand());
       mDriverController.a().whileTrue(mShooterSubsystem.runShooterPowerCommand());

       mDriverController.x().toggleOnTrue(mShooterSubsystem.stop());


        // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // cancelling on release.
        //mDriverController.b().whileTrue(mFlywheelSubsystem.setVelocity(RPM.of(ShooterConstants.DEFAULT_TARGET_RPM)));
       // mDriverController.leftTrigger().whileTrue(mFlywheelSubsystem.set(0.3));



        // Bumpers: Kicker and Intake
        mDriverController.rightBumper().whileTrue(shoot());
        //mDriverController.rightTrigger().whileTrue(mShooterSubsystem.runKickerCommand());
       // mDriverController.leftBumper().whileTrue(mShooterSubsystem.runIntakeForwardCommand());
       // mDriverController.leftTrigger().whileTrue(mShooterSubsystem.runIntakeBackwardCommand());

        





        // --- INTAKE & KICKER ---
        mDriverController.rightTrigger().whileTrue(mShooterSubsystem.runKickerCommand());
        mDriverController.leftTrigger().whileTrue(mIntakeSubsystem.runIntakeForwardCommand());
        mDriverController.leftBumper().whileTrue(shootwithJam());

    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

   public Command shoot(){
    return Commands.parallel(           
    new RunCommand(() -> mIntakeSubsystem.runIntake(Constants.IntakeConstants.INTAKE_SPEED)),
    new RunCommand(() -> mShooterSubsystem.runKicker(-Constants.ShooterConstants.KICKER_SPEED)));
  }
  public Command shootwithJam(){
    return Commands.parallel(           
    new UnjamCommand(mIntakeSubsystem),
    mShooterSubsystem.runKickerCommand());
  }


}