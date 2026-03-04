package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class UnjamIntakeCommand extends SequentialCommandGroup {

  public UnjamIntakeCommand(IntakeSubsystem mIntakeSubsystem) {
    addCommands(       
      mIntakeSubsystem.runOuttakeCommand().withTimeout(0.15),
      new WaitCommand(0.1),
      mIntakeSubsystem.runIntakeForwardCommand().withTimeout(1),
      new WaitCommand(0.1),
      mIntakeSubsystem.runOuttakeCommand().withTimeout(0.15),
      new WaitCommand(0.1),
      mIntakeSubsystem.runIntakeForwardCommand().withTimeout(1));
  }
}