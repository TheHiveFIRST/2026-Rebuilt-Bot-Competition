package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlignToTagCommand extends Command {

    public AutoAlignToTagCommand(DriveSubsystem mDriveSubsystem, CommandXboxController controller){
        new RunCommand( () -> mDriveSubsystem.driveJoystick(
            MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DRIVE_DEADBAND),
            LimelightHelpers.getTX("limelight")* -DriveConstants.AUTO_ALIGN_PID, 
            true), mDriveSubsystem);
    }
  
}
