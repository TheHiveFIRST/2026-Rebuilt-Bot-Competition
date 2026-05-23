package frc.robot.commands;


//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.HalfHopperIntakeWobbleCommand;

public class FullHopperIntakeWobbleCommand extends SequentialCommandGroup {
     public FullHopperIntakeWobbleCommand(ArmSubsystem mArmSubsystem, IntakeSubsystem mIntakeSubsystem) {
        addCommands(
            mArmSubsystem.IntakeSlightlyUpCommand().withTimeout(0.3),
            new WaitCommand(0.1),
            mArmSubsystem.IntakeSlightlyUpCommand().withTimeout(0.3),
            new WaitCommand(0.1),
            mArmSubsystem.IntakeSlightlyUpCommand().withTimeout(0.3),
            new WaitCommand(0.1),
            mArmSubsystem.IntakeSlightlyUpCommand().withTimeout(0.3),
            new WaitCommand(0.1),
            mArmSubsystem.IntakeSlightlyUpCommand().withTimeout(0.3),
            new WaitCommand(0.1),
            mArmSubsystem.IntakeSlightlyUpCommand().withTimeout(0.3),
            new WaitCommand(0.1),
            new HalfHopperIntakeWobbleCommand(mArmSubsystem, mIntakeSubsystem).repeatedly()
        );
     }
    
}
