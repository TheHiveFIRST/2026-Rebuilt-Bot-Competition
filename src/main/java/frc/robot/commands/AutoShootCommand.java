package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class AutoShootCommand extends SequentialCommandGroup {

  public AutoShootCommand(ShooterSubsystem mShooterSubsystem) {
    addCommands( 
    mShooterSubsystem.runShooterCommand().withTimeout(2), //TODO: change command to working shooter pidf 
    new WaitCommand(0.2),
    mShooterSubsystem.runKickerCommand().withTimeout(1));        
  }
}