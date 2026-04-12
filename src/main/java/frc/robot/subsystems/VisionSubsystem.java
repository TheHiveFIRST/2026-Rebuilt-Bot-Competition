package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {

     public VisionSubsystem(){ 

    }

    public double autoAlignRotationSpeed(){
        double autoRotSpeed = DriveConstants.AUTO_ALIGN_PID*getTX();
        return autoRotSpeed; 
    }

    public double getTX(){
        return LimelightHelpers.getTX("limelight");
    }

    public static LimelightHelpers.PoseEstimate getBotPoseEstimateBlue(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }


    @Override
    public void periodic(){
    SmartDashboard.putNumber("Testing/targetX", getTX());
          
   }
}
    
