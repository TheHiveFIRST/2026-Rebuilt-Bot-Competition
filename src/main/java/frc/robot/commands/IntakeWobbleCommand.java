package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class IntakeWobbleCommand extends SequentialCommandGroup {

  public IntakeWobbleCommand(ArmSubsystem mArmSubsystem) {
    addCommands( 
      mArmSubsystem.IntakeToPosition(ArmConstants.PIVOT_AGITATE).withTimeout(0.5),
      new WaitCommand(0.1),          
      mArmSubsystem.runIntakePivotGround().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.IntakeToPosition(ArmConstants.PIVOT_AGITATE55).withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotGround().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.IntakeToPosition(ArmConstants.PIVOT_AGITATE45).withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotGround().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.IntakeToPosition((ArmConstants.PIVOT_AGITATE35)).withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotGround().withTimeout(0.5));
  }
}