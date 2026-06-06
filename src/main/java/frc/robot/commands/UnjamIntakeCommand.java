package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// Removed unused imports per coding conventions
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class UnjamIntakeCommand extends ParallelCommandGroup {

  public UnjamIntakeCommand(IntakeSubsystem mIntakeSubsystem, ArmSubsystem mArmSubsystem) {
    /**
     * Create an unjam command that runs the intake forward and bumps the arm
     * to dislodge any stuck game piece.
     */
    addCommands( 
      mIntakeSubsystem.runIntakeForwardCommand(),
      mArmSubsystem.runIntakePivotBumpCommand()
    );
      
  }
}