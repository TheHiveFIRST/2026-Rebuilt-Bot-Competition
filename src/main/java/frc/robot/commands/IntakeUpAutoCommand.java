package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class IntakeUpAutoCommand extends SequentialCommandGroup {

  public IntakeUpAutoCommand(ArmSubsystem mArmSubsystem) {
    addCommands( 
      new WaitCommand(1),        
      mArmSubsystem.IntakeSlightlyUpCommand());
  }
}