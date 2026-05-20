package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTapCommand extends ParallelCommandGroup {

  public IntakeTapCommand(IntakeSubsystem mIntakeSubsystem, ArmSubsystem mArmSubsystem) {
    addCommands( 
      mIntakeSubsystem.runIntakeForwardCommand(),
      mArmSubsystem.runIntakePivotGround()
    );
  }
}