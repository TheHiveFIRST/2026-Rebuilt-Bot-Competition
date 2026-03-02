package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class UnjamCommand extends SequentialCommandGroup {

  public UnjamCommand(ShooterSubsystem mShooterSubsystem) {
    addCommands(       
      mShooterSubsystem.runKickerBackwardCommand().withTimeout(0.15),
      new WaitCommand(0.1),
      mShooterSubsystem.runKickerCommand().withTimeout(1),
      new WaitCommand(0.1),
      mShooterSubsystem.runKickerBackwardCommand().withTimeout(0.15),
      new WaitCommand(0.1),
      mShooterSubsystem.runKickerCommand().withTimeout(1));
  }
}