package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class IntakeWobbleCommand extends SequentialCommandGroup {

  public IntakeWobbleCommand(ArmSubsystem mArmSubsystem) {
    addCommands( 
       
      mArmSubsystem.IntakeSlightlyUp().withTimeout(0.5),
      new WaitCommand(0.1),
       mArmSubsystem.runIntakePivotGround().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.IntakeSlightlyUp().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotGround().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotUp().withTimeout(0.5),
      new WaitCommand(0.1),          
      mArmSubsystem.runIntakePivotGround().withTimeout(0.5)
    );
  }
}