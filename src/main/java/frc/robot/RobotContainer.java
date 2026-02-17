package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        mArmSubsystem.setDefaultCommand(mArmSubsystem.runIntakePivotGround());
        //mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runShooterPower(0), mShooterSubsystem));
        mIntakeSubsystem.setDefaultCommand(new RunCommand(()-> mIntakeSubsystem.runIntake(0), mIntakeSubsystem));
        mShooterSubsystem.setDefaultCommand(new RunCommand(()-> mShooterSubsystem.runKicker(0), mShooterSubsystem));
     }

    private void configureBindings() {
      
        //INTAKE CONTROLS 
        mOperatorController.a().whileTrue(mArmSubsystem.runIntakePivotGround());
        mOperatorController.b().whileTrue(mArmSubsystem.runIntakePivotUp());
        mOperatorController.y().whileTrue(mArmSubsystem.runPivot());
        
        mOperatorController.povLeft().onTrue(mArmSubsystem.runOnce(mArmSubsystem::incrementKP));
        mOperatorController.povRight().onTrue(mArmSubsystem.runOnce(mArmSubsystem::decrementKP));
        

        //SHOOTER CONTROLS
        mDriverController.y().whileTrue(mShooterSubsystem.runShooterCommand());
        mDriverController.a().whileTrue(mArmSubsystem.runIntakePivotUp());
        mDriverController.x().toggleOnTrue(mShooterSubsystem.stop());
        mDriverController.rightBumper().whileTrue(shoot());
        mDriverController.rightTrigger().whileTrue(mShooterSubsystem.runKickerCommand());
        mDriverController.leftBumper().whileTrue(shootwithJam());
        mDriverController.leftTrigger().whileTrue(mIntakeSubsystem.runIntakeForwardCommand());
        mDriverController.b().whileTrue(mDriveSubsystem.resetGyro()); 


        //  //schedule defense position when driver controller right bumper is pressed 
        //   mDriverController.leftTrigger()
        //  .whileTrue(mDriveSubsystem.defensePosition());
    
       
        //PID TUNING
        // Back Button (double tap to cycle)
       //mDriverController.back()
           // .whileTrue(mShooterSubsystem.run(mShooterSubsystem::cycleTuningMode))
           // .debounce(0.3); // Prevents accidental double presses
        
        mDriverController.povLeft().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementRPM));
        mDriverController.povRight().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));
        mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementCurrentGain));
        mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementCurrentGain));
        mDriverController.start().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementKP));
        mDriverController.back().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementKP));

        

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

   public Command shoot(){
    return Commands.parallel(           
    new RunCommand(() -> mIntakeSubsystem.runIntake(Constants.IntakeConstants.INTAKE_SPEED)),
    new RunCommand(() -> mShooterSubsystem.runKicker(-Constants.ShooterConstants.KICKER_SPEED)));
  }
  public Command shootwithJam(){
    return Commands.parallel(           
    new UnjamCommand(mIntakeSubsystem),
    mShooterSubsystem.runKickerCommand());
  }

}

  


