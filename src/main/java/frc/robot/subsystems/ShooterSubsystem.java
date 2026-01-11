package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ShooterSubsystem {
    // Variables
    private int leftShooterLeaderCanId = 1;
    private int rightShooterLeaderCanId = 2;
    SparkMax leftShooterLeader;
    SparkMax rightShooterLeader;

    // Constructor
    public ShooterSubsystem (){
        leftShooterLeader = new SparkMax(leftShooterLeaderCanId, MotorType.kBrushed);
        rightShooterLeader = new SparkMax(rightShooterLeaderCanId, MotorType.kBrushed);
    }
    
    // Methods
    public void shoot() {
        leftShooterLeader.set(1);
        rightShooterLeader.set(1);
    }

}
