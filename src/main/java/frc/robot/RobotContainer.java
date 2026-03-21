package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoAlignToTagCommand;
import frc.robot.commands.AutonAlignCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeTapCommand;
import frc.robot.commands.IntakeWobbleCommand;
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

import frc.robot.commands.UnjamKickerCommand;
import frc.robot.commands.UnjamIntakeCommand;
import frc.robot.configs.DriveConfig;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;



import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem(); 
  private final ArmSubsystem mArmSubsystem = new ArmSubsystem(); 
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem(); 
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem(); 
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController mDriverController = 
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);
  private final CommandXboxController mOperatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);
  boolean slowMode = false;


  public RobotContainer() {
    // Register named commands
    //NamedCommands.registerCommand("intakepivotdefault", mArmSubsystem.IntakePivotDefault());
    NamedCommands.registerCommand("intakepivotup", mArmSubsystem.runIntakePivotUp());
    NamedCommands.registerCommand("intakepivotdown", new IntakeTapCommand(mIntakeSubsystem, mArmSubsystem).withTimeout(3));
    NamedCommands.registerCommand("shoot", autoShoot().withTimeout(10));
    NamedCommands.registerCommand("rampupshoot", mShooterSubsystem.toggleAutoShooterCommand().withTimeout(3));
    NamedCommands.registerCommand("stopshoot", autoStopShoot());
    NamedCommands.registerCommand("autoalign", new AutonAlignCommand(mDriveSubsystem));


    // new EventTrigger("shoot").onTrue(autoShoot());
    // new EventTrigger("stopshoot").onTrue(autoStopShoot());
    // new EventTrigger("intakepivotdefault").onTrue(mArmSubsystem.IntakePivotDefault());

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser).withSize(2, 1);
    
    
    configureBindings();
    mDriveSubsystem.setDefaultCommand(  
    new RunCommand(
            () -> {
            double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
            mDriveSubsystem.driveJoystick(
                MathUtil.applyDeadband(mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                true);},
            mDriveSubsystem));

        mArmSubsystem.setDefaultCommand(mArmSubsystem.runIntakePivotGround());
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runShooterPower(0), mShooterSubsystem));
        mIntakeSubsystem.setDefaultCommand(new RunCommand(()-> mIntakeSubsystem.runIntake(0), mIntakeSubsystem));
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runKicker(0), mShooterSubsystem));

        SmartDashboard.putData("setShooterSpeeds", new InstantCommand(() -> mShooterSubsystem.setSpeedsSmartDashboard()));

     }

    private void configureBindings() {
      
        //OPERATOR CONTROLS
        mOperatorController.y().whileTrue(mShooterSubsystem.toggleShooterCommand());
        mOperatorController.leftTrigger().onTrue(mShooterSubsystem.setLadderShotCommand());
        mOperatorController.rightTrigger().onTrue(mShooterSubsystem.setPassingShotCommand());
        mOperatorController.b().onTrue(mShooterSubsystem.setHubShotCommand());
        mOperatorController.a().onTrue(mShooterSubsystem.setTrenchShotCommand());
        mOperatorController.x().whileTrue(mArmSubsystem.toggleArmStageCommand());
        mOperatorController.rightBumper().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mOperatorController.leftBumper().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());
        mOperatorController.povLeft().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementRPM));
        mOperatorController.povRight().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));


         //DRIVER CONTROLS
        mDriverController.y().whileTrue(mShooterSubsystem.toggleShooterCommand()); 
        //mDriverController.a().whileTrue(new AutoAlignToTagCommand(mDriveSubsystem, mDriverController));
        mDriverController.x().whileTrue(shootWithAgitation());
        mDriverController.b().onTrue(mIntakeSubsystem.runOuttakeCommand());

        mDriverController.rightBumper().whileTrue(mShooterSubsystem.runKickerCommand());
        mDriverController.rightTrigger().whileTrue(shootWithAgitation());
        mDriverController.leftBumper().whileTrue(mArmSubsystem.runIntakePivotUp());
        mDriverController.leftTrigger().whileTrue(mIntakeSubsystem.runIntakeForwardCommand());
        mDriverController.leftStick().whileTrue(mDriveSubsystem.defensePosition());

        mDriverController.start().whileTrue(mDriveSubsystem.resetGyro()); 
        mDriverController.povLeft().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mDriverController.povRight().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());
        mDriverController.povDown().onTrue(toggleSlowMode());
        mDriverController.povUp().whileTrue(mIntakeSubsystem.runIntakeSlowCommand());
       // mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::incrementPalign));
       // mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::decrementPalign));
        //mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));


         mDriverController.a().whileTrue(new RunCommand(
         () -> mDriveSubsystem.driveJoystick(
           MathUtil.applyDeadband(mDriverController.getLeftY(), OperatorConstants.DRIVE_DEADBAND),
           MathUtil.applyDeadband(mDriverController.getLeftX(), OperatorConstants.DRIVE_DEADBAND),
          LimelightHelpers.getTX("limelight")* -DriveConstants.AUTO_ALIGN_PID, 
          true), mDriveSubsystem));
        //intake forward align 
        // //TODO: test if this \works 
        // mDriverController.leftTrigger().whileTrue(
        //     new RunCommand(
        //        () -> mDriveSubsystem.driveIntakeAlign(
        //          mDriverController.getLeftY(), 
        //          mDriverController.getLeftX(),
        //       true), mDriveSubsystem));

        //diagonal bump align 
        //TODO: test if this works 
        // mDriverController.rightTrigger().whileTrue(
        //     new RunCommand(
        //        () -> mDriveSubsystem.driveDiagonalBumpAlign(
        //          mDriverController.getLeftY(), 
        //          mDriverController.getLeftX(),
        //         true, 
        //         135), mDriveSubsystem));

       
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

    
    public Command shootWithAgitation(){
        return Commands.parallel(           
        new IntakeWobbleCommand(mArmSubsystem),
        new UnjamIntakeCommand(mIntakeSubsystem),
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
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  public Command autoStopShoot(){
    return Commands.parallel(           
    new RunCommand(() -> mShooterSubsystem.runKicker(0)),
    new RunCommand(() -> mShooterSubsystem.runShooterPower(0)));
    
  }

    
  public Command autoShoot(){
    return Commands.parallel(           
    new RunCommand(() -> mShooterSubsystem.runKicker(-ShooterConstants.KICKER_SPEED)),
    new RunCommand(() -> mShooterSubsystem.toggleAutoShooterCommand()));
  }
}

  


