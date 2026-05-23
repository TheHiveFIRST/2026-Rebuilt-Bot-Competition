package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FullHopperIntakeWobbleCommand;
import frc.robot.commands.IntakeTapCommand;
import frc.robot.commands.HalfHopperIntakeWobbleCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
  private final VisionSubsystem mVisionSubsystem = new VisionSubsystem(); 
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
    NamedCommands.registerCommand("intakepivotslightlyup", mArmSubsystem.IntakeSlightlyUp().withTimeout(1.80));
    NamedCommands.registerCommand("intakepivotup", mArmSubsystem.IntakeToPosition(0.55).withTimeout(8));
    NamedCommands.registerCommand("runintake", mIntakeSubsystem.runIntakeForwardCommand().withTimeout(5));

    NamedCommands.registerCommand("intakepivotdown", new IntakeTapCommand(mIntakeSubsystem, mArmSubsystem).withTimeout(2.93));
    NamedCommands.registerCommand("shoot", mShooterSubsystem.setHubShotCommand().andThen(
                                                mShooterSubsystem.toggleAutoShooterCommand().andThen(
                                                autoWobbleShoot())));
    NamedCommands.registerCommand("runkicker", mShooterSubsystem.runKickerCommand().withTimeout(3));                                             
    NamedCommands.registerCommand("setshot", mShooterSubsystem.toggleAutoShooterCommand().withTimeout(8));
    NamedCommands.registerCommand("stopshoot", autoStopKicker());

   
     new EventTrigger("setshot").onTrue(mShooterSubsystem.toggleAutoShooterCommand().withTimeout(3));
     new EventTrigger("intakepivotdown2").whileTrue(new IntakeTapCommand(mIntakeSubsystem, mArmSubsystem));


    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser).withSize(2, 1);
    
    
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
     }

    public void configureJoysticks(String pickedAuto) {
      switch (pickedAuto) {
        case "DOUBLE_SWIPE_HUMAN_PLAYER":
        case "SINGLE_SWIPE_HUMAN_PLAYER":
          DriveSubsystem.gyrooffset = -90;
          mDriveSubsystem.setDefaultCommand(
          new RunCommand(() -> {
              double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
              mDriveSubsystem.driveJoystick(
                  MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  true);},
              mDriveSubsystem));
          
          break;
        case "DOUBLE_SWIPE_DEPOT":
        case "SINGLE_SWIPE_DEPOT":
          DriveSubsystem.gyrooffset = 90;
          mDriveSubsystem.setDefaultCommand(
          new RunCommand(() -> {
              double currentDriveSpeed = slowMode ? DriveConstants.DRIVE_SPEED * DriveConstants.SLOW_MODE_MULTIPLIER : DriveConstants.DRIVE_SPEED;
              mDriveSubsystem.driveJoystick(
                  MathUtil.applyDeadband(-mDriverController.getLeftX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getLeftY()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  MathUtil.applyDeadband(-mDriverController.getRightX()*currentDriveSpeed, OperatorConstants.DRIVE_DEADBAND),
                  true);},
              mDriveSubsystem));
          break;
        default: // started in front of the hub
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
          break;
      }
    }
    private void configureBindings() {
      
        //OPERATOR CONTROLS
        mOperatorController.y().whileTrue(mShooterSubsystem.toggleShooterCommand());
        mOperatorController.rightTrigger().onTrue(mShooterSubsystem.setLadderShotCommand());
        //mOperatorController.leftTrigger().onTrue(mShooterSubsystem.setPassingShotCommand());
        mOperatorController.b().onTrue(mShooterSubsystem.setHubShotCommand());
        mOperatorController.a().onTrue(mShooterSubsystem.setTrenchShotCommand());
        mOperatorController.leftTrigger().whileTrue(mDriveSubsystem.alignV2Drive(mDriverController, () -> DriveConstants.getHubPose().toPose2d()));
        mOperatorController.x().onTrue(mShooterSubsystem.toggleDistanceEstimationCommand());
        mOperatorController.rightBumper().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mOperatorController.leftBumper().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());

        mOperatorController.povDown().whileTrue(mDriveSubsystem.characterizeAngular(3));
        mOperatorController.povUp().whileTrue(mDriveSubsystem.characterizeLinear(4));
        //mOperatorController.povLeft().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::incrementRPM));
        //mOperatorController.povRight().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));
        //mOperatorController.leftStick().whileTrue(mDriveSubsystem.defensePosition());


         //DRIVER CONTROLS
        mDriverController.y().whileTrue(mShooterSubsystem.toggleShooterCommand()); 
        mDriverController.x().whileTrue(mShooterSubsystem.runKickerBackwardCommand());
        mDriverController.b().onTrue(mIntakeSubsystem.runOuttakeCommand());

        mDriverController.rightBumper().whileTrue(shootHalfHopper());
        mDriverController.rightTrigger().whileTrue(shootFullHopper());
        mDriverController.leftBumper().whileTrue(mArmSubsystem.runIntakePivotUp());
        mDriverController.leftTrigger(0.2).whileTrue(mIntakeSubsystem.runIntakeForwardCommand());
 
        mDriverController.start().whileTrue(mDriveSubsystem.resetGyro()); 
        
        mDriverController.povLeft().onTrue(mShooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        mDriverController.povRight().onTrue(mShooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        mDriverController.povDown().onTrue(mShooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        mDriverController.povUp().onTrue(mShooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        /*mDriverController.povLeft().onTrue(mShooterSubsystem.increaseShootingRPMOffsetCommand());
        mDriverController.povRight().onTrue(mShooterSubsystem.decreaseShootingRPMOffsetCommand());
        mDriverController.povDown().onTrue(toggleSlowMode());
        mDriverController.povUp().whileTrue(mIntakeSubsystem.runIntakeSlowCommand());*/
       // mDriverController.povUp().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::incrementPalign));
       // mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mDriveSubsystem::decrementPalign));
        //mDriverController.povDown().onTrue(mShooterSubsystem.runOnce(mShooterSubsystem::decrementRPM));


         mDriverController.a().whileTrue(new RunCommand(
         () -> mDriveSubsystem.driveJoystick(
           MathUtil.applyDeadband(-mDriverController.getLeftY(), OperatorConstants.DRIVE_DEADBAND),
           MathUtil.applyDeadband(-mDriverController.getLeftX(), OperatorConstants.DRIVE_DEADBAND),
          -mVisionSubsystem.autoAlignRotationSpeed(), 
          true), mDriveSubsystem));
    

       
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
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
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

  


