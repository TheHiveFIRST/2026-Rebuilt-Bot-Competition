package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class UnjamIntakeCommand extends ParallelCommandGroup {

  public UnjamIntakeCommand(IntakeSubsystem mIntakeSubsystem, ArmSubsystem mArmSubsystem) {
    addCommands( 
      mIntakeSubsystem.runIntakeForwardCommand(),
      mArmSubsystem.runIntakePivotUp()
    );
      
  }
}