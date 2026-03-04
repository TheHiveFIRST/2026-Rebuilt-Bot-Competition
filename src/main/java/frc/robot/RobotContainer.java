// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final SwerveSubsystem swerve = new SwerveSubsystem();

  private final SendableChooser<Command> autoChooser;
  private final DriveSubsystem swerve = new DriveSubsystem(); 
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem(); 
  private final ArmSubsystem mArmSubsystem = new ArmSubsystem(); 


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController mDriverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("intakepivotdefault", mArmSubsystem.IntakePivotDefault());
    NamedCommands.registerCommand("shoot", autoShoot());
    NamedCommands.registerCommand("rampupshoot", mShooterSubsystem.runShooterAutoCommand().withTimeout(4));
    NamedCommands.registerCommand("stopshoot", autoStopShoot());

    new EventTrigger("shoot").onTrue(autoShoot());
    new EventTrigger("stopshoot").onTrue(autoStopShoot());
    new EventTrigger("intakepivotdefault").onTrue(mArmSubsystem.IntakePivotDefault());


    // Use event markers as triggers
    new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));

    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    Shuffleboard.getTab("Autonomous").add("Auto Mode", autoChooser).withSize(2, 1);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above

    // Add a button to run pathfinding commands to SmartDashboard
   

     
    };
  

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
    new RunCommand(() -> mShooterSubsystem.runKicker(-ShooterConstants.KICKER_SPEED)).withTimeout(5),
    new RunCommand(() -> mShooterSubsystem.runShooterAutoCommand()).withTimeout(5));
  }
}
