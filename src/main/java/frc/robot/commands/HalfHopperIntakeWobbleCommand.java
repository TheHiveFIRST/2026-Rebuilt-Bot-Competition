package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class HalfHopperIntakeWobbleCommand extends SequentialCommandGroup {

  public HalfHopperIntakeWobbleCommand(ArmSubsystem mArmSubsystem, IntakeSubsystem mIntakeSubsystem) {
    addCommands( 
      new SequentialCommandGroup(
        mArmSubsystem.runIntakePivotUp().withTimeout(0.3),
        new WaitCommand(0.1),
        mArmSubsystem.runIntakePivotGround().withTimeout(0.3),
        new WaitCommand(0.1)
    ).deadlineWith(
        mIntakeSubsystem.runIntakeForwardCommand()
    ).repeatedly()
      
    
       
      
      
      );
  }
}