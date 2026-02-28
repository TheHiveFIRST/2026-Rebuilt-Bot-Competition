package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoAlignToTagCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeTapCommand;
import frc.robot.commands.IntakeWobbleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
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

import frc.robot.commands.UnjamCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;




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


  public RobotContainer() {
        configureBindings();

        mDriveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick. field relative set true/false 
        new RunCommand(
            () -> mDriveSubsystem.driveJoystick(
                MathUtil.applyDeadband(mDriverController.getLeftY(), OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(mDriverController.getLeftX(), OperatorConstants.DRIVE_DEADBAND),
                MathUtil.applyDeadband(mDriverController.getRightX(), OperatorConstants.DRIVE_DEADBAND),
                true),
            mDriveSubsystem));

        mArmSubsystem.setDefaultCommand(mArmSubsystem.IntakePivotDefault());
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runShooterPower(0), mShooterSubsystem));
        mIntakeSubsystem.setDefaultCommand(new RunCommand(()-> mIntakeSubsystem.runIntake(0), mIntakeSubsystem));
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runKicker(0), mShooterSubsystem));

        SmartDashboard.putData("setShooterSpeeds", new InstantCommand(() -> mShooterSubsystem.setSpeedsSmartDashboard()));

     }

    private void configureBindings() {
      
        //INTAKE CONTROLS 
        // mOperatorController.a().whileTrue(mArmSubsystem.runIntakePivotGround());
        // mOperatorController.b().whileTrue(mArmSubsystem.runIntakePivotUp());
        mOperatorController.leftBumper().whileTrue(mShooterSubsystem.toggleShooterCommand());
        mOperatorController.leftTrigger().toggleOnTrue(mShooterSubsystem.stop());
        mOperatorController.a().onTrue(mShooterSubsystem.setHubShotCommand());
        mOperatorController.b().onTrue(mShooterSubsystem.setTrenchShotCommand());
        mOperatorController.povUp().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mOperatorController.povDown().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());
        mOperatorController.povLeft().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementRPM));
        mOperatorController.povRight().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));


        //mOperatorController.y().whileTrue(mArmSubsystem.runPivot());
        
        //mOperatorController.povLeft().onTrue(mArmSubsystem.runOnce(mArmSubsystem::incrementKP));
        //mOperatorController.povRight().onTrue(mArmSubsystem.runOnce(mArmSubsystem::decrementKP));
        
         //SHOOTER CONTROLS
        mDriverController.y().whileTrue(mShooterSubsystem.toggleShooterCommand()); 
        //mDriverController.a().whileTrue(new AutoAlignToTagCommand(mDriveSubsystem, mDriverController));
        mDriverController.x().toggleOnTrue(mShooterSubsystem.stop());

        mDriverController.rightBumper().whileTrue(mShooterSubsystem.runKickerCommand());
        mDriverController.rightTrigger().whileTrue(mShooterSubsystem.runKickerBackwardCommand());
        mDriverController.leftBumper().whileTrue(mArmSubsystem.runIntakePivotUp());
        mDriverController.leftTrigger().whileTrue(new IntakeTapCommand(mIntakeSubsystem, mArmSubsystem));
        
        mOperatorController.a().onTrue(mShooterSubsystem.setHubShotCommand());
        mOperatorController.back().onTrue(mShooterSubsystem.setTrenchShotCommand());

        mDriverController.start().whileTrue(mDriveSubsystem.resetGyro()); 
        mDriverController.povLeft().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mDriverController.povRight().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());
       // mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementRPM));
       // mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));

        //intake forward align 
        // //TODO: test if this works 
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

    public Command getAutonomousCommand() {
        return Commands.none();
    }

//    public Command shoot(){
//     return Commands.parallel(           
//     new RunCommand(() -> mIntakeSubsystem.runIntake(Constants.IntakeConstants.INTAKE_SPEED)),
//     new RunCommand(() -> mShooterSubsystem.runKicker(-Constants.ShooterConstants.KICKER_SPEED)));
//   }
//  
  
    
    public Command shootWobble(){
        return Commands.parallel(           
        new IntakeWobbleCommand(mArmSubsystem),
        mShooterSubsystem.runKickerCommand());
    }


}

  


