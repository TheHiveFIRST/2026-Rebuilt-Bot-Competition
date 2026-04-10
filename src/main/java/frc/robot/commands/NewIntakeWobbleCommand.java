package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class NewIntakeWobbleCommand extends SequentialCommandGroup {

  public NewIntakeWobbleCommand(ArmSubsystem mArmSubsystem, IntakeSubsystem mIntakeSubsystem) {
    addCommands( 
      mArmSubsystem.IntakeSlightlyUp().withTimeout(0.5),
      new WaitCommand(0.1),          
      mArmSubsystem.runIntakePivotGround().withTimeout(0.5),
      new WaitCommand(0.1),
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
      
       mArmSubsystem.runIntakePivotGround().withTimeout(0.)
       
      
      //mIntakeSubsystem.stop()
      );
  }
}