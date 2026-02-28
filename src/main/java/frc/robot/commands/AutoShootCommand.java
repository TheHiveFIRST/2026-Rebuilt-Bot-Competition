// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.ShooterSubsystem;

// /** A complex auto command that drives forward, releases a hatch, and then drives backward. */
// public class AutoShootCommand extends ParallelCommandGroup {

//   public AutoShootCommand(ShooterSubsystem mShooterSubsystem) {
//     addCommands( 
//     mShooterSubsystem.runShooterAutoCommand().withTimeout(3), //TODO: change command to working shooter pidf 
//     mShooterSubsystem.runKickerCommand().withTimeout(3));        
//   }

// }