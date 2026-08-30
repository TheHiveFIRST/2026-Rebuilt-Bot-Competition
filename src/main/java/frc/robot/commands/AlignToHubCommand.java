// // src/main/java/frc/robot/commands/AlignToHub.java
// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.subsystems.DriveSubsystem;

// public class AlignToHubCommand extends Command {
//     private final DriveSubsystem drive;
//     private final DoubleSupplier xSupplier;   
//     private final DoubleSupplier ySupplier;   

//     private final ProfiledPIDController headingController;


//     public AlignToHubCommand(
//             DriveSubsystem drive,
//             DoubleSupplier xSupplier,
//             DoubleSupplier ySupplier) {

//         this.drive = drive;
//         this.xSupplier = xSupplier;
//         this.ySupplier = ySupplier;

//         this.headingController = new ProfiledPIDController(
//                 DriveConstants.ROTATION_KP,
//                 0.0,
//                 0.0,
//                 new TrapezoidProfile.Constraints(
//                         DriveConstants.MAX_ANGULAR_SPEED,          
//                         DriveConstants.MAX_ANGULAR_SPEED * 2.0)); 

//         headingController.enableContinuousInput(-Math.PI, Math.PI);
//         headingController.setTolerance(Math.toRadians(2.0)); 

//         addRequirements(drive);
//     }

//     @Override
//     public void initialize() {
//         double currentRad = drive.getPose().getRotation().getRadians();
//         headingController.reset(currentRad);
//     }

//     @Override
//     public void execute() {
//         double vx = MathUtil.applyDeadband(xSupplier.getAsDouble(), OperatorConstants.DRIVE_DEADBAND);
//         double vy = MathUtil.applyDeadband(ySupplier.getAsDouble(), OperatorConstants.DRIVE_DEADBAND);

//         Pose2d pose = drive.getPose();
//         Translation2d hub = DriveConstants.getHubPose().toPose2d().getTranslation();

//         Translation2d robotToHub = hub.minus(pose.getTranslation());
//         Rotation2d desired = robotToHub.getAngle();

//         double rotOutput = headingController.calculate(
//                 pose.getRotation().getRadians(),
//                 desired.getRadians());

//         drive.driveJoystick(vx, vy, rotOutput, true);
        
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }
// }