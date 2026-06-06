package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;


/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class IntakeWobbleCommand extends SequentialCommandGroup {

  public IntakeWobbleCommand(ArmSubsystem mArmSubsystem) {
    addCommands( 
      mArmSubsystem.IntakeToPositionCommand(ArmConstants.PIVOT_AGITATE).withTimeout(0.5),
      new WaitCommand(0.1),          
      mArmSubsystem.runIntakePivotGroundCommand().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.IntakeToPositionCommand(ArmConstants.PIVOT_AGITATE55).withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotGroundCommand().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.IntakeToPositionCommand(ArmConstants.PIVOT_AGITATE45).withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotGroundCommand().withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.IntakeToPositionCommand((ArmConstants.PIVOT_AGITATE35)).withTimeout(0.5),
      new WaitCommand(0.1),
      mArmSubsystem.runIntakePivotGroundCommand().withTimeout(0.5));
  }
}