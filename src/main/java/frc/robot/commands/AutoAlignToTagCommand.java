package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlignToTagCommand extends Command {

    public AutoAlignToTagCommand(DriveSubsystem mDriveSubsystem, CommandXboxController controller){
        new RunCommand( () -> mDriveSubsystem.driveJoystick(
                controller.getLeftY(), 
                controller.getLeftX(), 
                LimelightHelpers.getTX("limelight")* 1.0, 
                true), mDriveSubsystem);
    }
  
}
