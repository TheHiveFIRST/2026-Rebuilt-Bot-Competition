package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;



import frc.robot.commands.FullHopperIntakeWobbleCommand;



import frc.robot.commands.HalfHopperIntakeWobbleCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;





import frc.robot.configs.DriveConfig;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.subsystems.IntakeSubsystem;



import java.util.List;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(); 
  private final ArmSubsystem mArmSubsystem = new ArmSubsystem(); 
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem(); 
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem(); 
  

  private final CommandXboxController mDriverController = 
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController mOperatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);
  boolean slowMode = false;


//   public Pose2d getAllianceStartingPose() {
//     if (Constants.getCurrentAlliance() == Alliance.Blue) {
//         return new Pose2d(
//             new Translation2d(2.7432, 3.2512),  
//             Rotation2d.fromDegrees(0)
//         );
//     } else {
//         return new Pose2d(
//             new Translation2d(16.0, 4.0),  
//             Rotation2d.fromDegrees(180)
//         );
//     }
// } 


  public RobotContainer() {
    // Register named commands
   
   

    // new EventTrigger("shoot").onTrue(autoShoot());
    // new EventTrigger("stopshoot").onTrue(autoStopShoot());
   
    


    
    
    
    configureBindings();
    mDriveSubsystem.setDefaultCommand(  
    new RunCommand(
            () -> {
            double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
            mDriveSubsystem.driveJoystick(
                MathUtil.applyDeadband(-mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                true);},
            mDriveSubsystem));

        mArmSubsystem.setDefaultCommand(mArmSubsystem.runIntakePivotGround());
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runShooterPower(0), mShooterSubsystem));
        mIntakeSubsystem.setDefaultCommand(new RunCommand(()-> mIntakeSubsystem.runIntake(0), mIntakeSubsystem));
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runKicker(0), mShooterSubsystem));

       //SmartDashboard.putData("setShooterSpeeds", new InstantCommand(() -> mShooterSubsystem.setSpeedsSmartDashboard()));

     }
    public void configureJoysticks(String pickedAuto) {
      
        
          DriveSubsystem.gyrooffset = 0;
          mDriveSubsystem.setDefaultCommand(  
            new RunCommand(() -> {
              double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
              mDriveSubsystem.driveJoystick(
                  MathUtil.applyDeadband(-mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  true);},
              mDriveSubsystem));
          
    }
    private void configureBindings() {
      
        


         //DRIVER CONTROLS
        mDriverController.y().whileTrue(mShooterSubsystem.toggleShooterCommand()); 
        mDriverController.x().whileTrue(mShooterSubsystem.runKickerBackwardCommand());
        mDriverController.b().onTrue(mIntakeSubsystem.runOuttakeCommand());

        mDriverController.rightBumper().whileTrue(shootHalfHopper());
        mDriverController.rightTrigger().whileTrue(shootFullHopper());
        mDriverController.leftBumper().whileTrue(mArmSubsystem.runIntakePivotUp());
        mDriverController.leftTrigger(0.2).whileTrue(mIntakeSubsystem.runIntakeForwardCommand());
        

        mDriverController.start().whileTrue(mDriveSubsystem.resetGyro()); 
        mDriverController.povLeft().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mDriverController.povRight().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());
        mDriverController.povDown().onTrue(toggleSlowMode());
        mDriverController.povUp().whileTrue(mIntakeSubsystem.runIntakeSlowCommand());
       // mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::incrementPalign));
       // mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::decrementPalign));
        //mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));


        
    

       
        //PID TUNING
        // Back Button (double tap to cycle)
       //mDriverController.back()
           // .whileTrue(mShooterSubsystem.run(mShooterSubsystem::cycleTuningMode))
           // .debounce(0.3); // Prevents accidental double presses
        
        //mDriverController.start().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementCurrentGain));
        //mDriverController.back().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementCurrentGain));
        //mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementKP));
        //mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementKP));

        

        // --- ARM / STINGER (A Button Toggle) ---
        /* 
        mDriverController.a().onTrue(
            Commands.runOnce(() -> {
                // If current position is near the OUT setpoint, move to IN. Otherwise, move OUT.
                if (Math.abs(mArmSubsystem.encoderGetValue() - ArmConstants.PIVOT_OUT) < 0.05) {
                    mArmSubsystem.setTargetArm(ArmConstants.PIVOT_IN);
                } else {
                    mArmSubsystem.setTargetArm(ArmConstants.PIVOT_OUT);
                }
            }, mArmSubsystem)
        );
        */

    }

    public void zeroGyroHeading(){
      mDriveSubsystem.zeroHeading();
    }

    // public void resetVisionPose(){
    //   mDriveSubsystem.resetPoseEstimator(getAllianceStartingPose());
    // }

    
    public Command shootFullHopper(){
      return Commands.parallel(           
      new FullHopperIntakeWobbleCommand(mArmSubsystem, mIntakeSubsystem),
      mDriveSubsystem.defensePosition(),
      
      mShooterSubsystem.runKickerCommand());
  
    }
    public Command shootHalfHopper(){
      return Commands.parallel(           
      new HalfHopperIntakeWobbleCommand(mArmSubsystem, mIntakeSubsystem),
      mDriveSubsystem.defensePosition(),
      
      mShooterSubsystem.runKickerCommand());
  
    }
    public Command toggleSlowMode(){
        return new InstantCommand(() -> slowMode = !slowMode);
    }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  
  public Command autoStopKicker (){
    return new InstantCommand(() -> mShooterSubsystem.runKicker(0));
  }

    
  public Command autoShoot(){
    return Commands.parallel(           
    mShooterSubsystem.runKickerCommand(),
    new FullHopperIntakeWobbleCommand(mArmSubsystem, mIntakeSubsystem)
    );
  }

   public Command autoWobbleShoot(){
   return Commands.parallel(   
    //new IntakeUpAutoCommand(mArmSubsystem, mIntakeSubsystem).repeatedly(),        
    mShooterSubsystem.runKickerCommand());
  }
}

  


