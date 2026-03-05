package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonAlignCommand extends Command {

    private final DriveSubsystem mDriveSubsystem;

    public AutonAlignCommand(DriveSubsystem mDriveSubsystem) {
        this.mDriveSubsystem = mDriveSubsystem;
        addRequirements(mDriveSubsystem);
    }

    @Override
    public void execute() {

        double tx = LimelightHelpers.getTX("limelight");

        double rot = MathUtil.clamp(
            tx * -DriveConstants.AUTO_ALIGN_PID,
            -DriveConstants.AUTO_ALIGN_MAX_SPEED,
            DriveConstants.AUTO_ALIGN_MAX_SPEED
        );

        mDriveSubsystem.driveJoystick(
            0, 
            0, 
            rot,
            false
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTX("limelight")) < 2.0; // tolerance
    }

    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.driveJoystick(0,0,0,false);
    }
}




//TODO:  2. elastic layout 