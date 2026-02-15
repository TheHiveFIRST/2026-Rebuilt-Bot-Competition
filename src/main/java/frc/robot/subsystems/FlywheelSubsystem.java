// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Feet;
// import static edu.wpi.first.units.Units.Pounds;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.RPM;
// import static edu.wpi.first.units.Units.Volts;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.configs.ShootConfig.ShooterConfig;
// import frc.robot.Constants.ShooterConstants;

// import yams.mechanisms.config.FlyWheelConfig;
// import yams.mechanisms.velocity.FlyWheel;
// import yams.gearing.GearBox;
// import yams.gearing.MechanismGearing;
// import yams.mechanisms.SmartMechanism;
// import yams.motorcontrollers.SmartMotorController;
// import yams.motorcontrollers.SmartMotorControllerConfig;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
// import yams.motorcontrollers.local.SparkWrapper;


// public class FlywheelSubsystem extends SubsystemBase {

//  private double mCurrentKV = ShooterConstants.FF_KV;
//  private double mCurrentKA = ShooterConstants.FF_KA;
//  private double mCurrentKP = ShooterConstants.TESTING_KP;
//  private double mCurrentKI = ShooterConstants.TESTING_KI;
//  private double mCurrentKD = ShooterConstants.TESTING_KD;
//  private final RelativeEncoder mShooterLeaderEncoder;




// // Vendor motor controller object
//   private SparkMax mShooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);

//   private SmartMotorControllerConfig shooterMotorConfig = new SmartMotorControllerConfig(this)
//   .withControlMode(ControlMode.CLOSED_LOOP)
//   // Feedback Constants (PID Constants)
//   .withClosedLoopController(mCurrentKP, mCurrentKI, mCurrentKD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
//   .withSimClosedLoopController(mCurrentKP, mCurrentKI, mCurrentKD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
//   // Feedforward Constants
//   .withFeedforward(new SimpleMotorFeedforward(0, mCurrentKV, mCurrentKA))
//   .withSimFeedforward(new SimpleMotorFeedforward(0, mCurrentKV, mCurrentKA))
//   // Telemetry name and verbosity level REMOVE LATER 
//   .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//   .withMotorInverted(false)
//   .withIdleMode(MotorMode.COAST)
//   .withStatorCurrentLimit(Amps.of(40))
//   .withGearing(1)
//   .withFollowers(Pair.of(new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless), true));
  

//   private SmartMotorController sparkSmartMotorController = new SparkWrapper(mShooterLeader, DCMotor.getNEO(2), shooterMotorConfig);

//   private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
//   // Diameter of the flywheel.
//   .withDiameter(Inches.of(3))
//   // Mass of the flywheel.
//   .withMass(Pounds.of(2.12))
//   // Maximum speed of the shooter.
//   .withUpperSoftLimit(RPM.of(5500))
//   .withLowerSoftLimit(RPM.of(0))
//   // Telemetry name and verbosity for the arm.
//   .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

//   // Shooter Mechanism
//   private FlyWheel shooter = new FlyWheel(shooterConfig);

//     /**
//    * Gets the current velocity of the shooter.
//    *
//    * @return Shooter velocity.
//    */
//   public AngularVelocity getVelocity() {return shooter.getSpeed();}

//   public double getAngularVelocity() { return mShooterLeaderEncoder.getVelocity() *Math.PI/30;  }
//    public double getRPM() { return mShooterLeaderEncoder.getVelocity();  }


//   /**
//    * Set the shooter velocity.
//    *
//    * @param speed Speed to set.
//    * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
//    */
//   public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}
  
//   /**
//    * Set the shooter velocity setpoint.
//    *
//    * @param speed Speed to set
//    */
//   public void setVelocitySetpoint(AngularVelocity speed) {shooter.setMechanismVelocitySetpoint(speed);}

//     public void incrementKD() { mCurrentKD += ShooterConstants.KD_INCREMENT; } // Increments by 0.01 for fine tuning
//     public void decrementKD() { mCurrentKD -= ShooterConstants.KD_INCREMENT; }

//     public void incrementKI() { mCurrentKI += ShooterConstants.KI_INCREMENT; } // Increments by 0.01 for fine tuning
//     public void decrementKI() { mCurrentKI -= ShooterConstants.KI_INCREMENT; }

//     public void incrementKP() { mCurrentKP += ShooterConstants.KP_INCREMENT; } // Increments by 0.01 for fine tuning
//     public void decrementKP() { mCurrentKP -= ShooterConstants.KP_INCREMENT; }
//   /**
//    * Set the dutycycle of the shooter.
//    *
//    * @param dutyCycle DutyCycle to set.
//    * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
//    */
//   public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

//   /** Creates a new ExampleSubsystem. */
//   public FlywheelSubsystem() {
//         mShooterLeaderEncoder = mShooterLeader.getEncoder();

//   }

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

  

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     // This method will be called once per scheduler run
//     shooter.updateTelemetry();
//     SmartDashboard.putNumber("Shooter/Target RPM", ShooterConstants.DEFAULT_TARGET_RPM);
//     SmartDashboard.putNumber("Shooter/Actual AngularVelocity", getAngularVelocity());
//     SmartDashboard.putNumber("Shooter/Actual RPM", getRPM());
//     SmartDashboard.putNumber("Shooter/Current kD Tuning", mCurrentKD);
//     SmartDashboard.putNumber("Shooter/Current kP Tuning", mCurrentKP);
//     SmartDashboard.putNumber("Shooter/Current kI Tuning", mCurrentKI);
//     SmartDashboard.putNumber("Shooter/intake bus voltage", mShooterLeader.getBusVoltage());
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//     // This method will be called once per scheduler run during simulation
//     shooter.simIterate();
//   }
// }
