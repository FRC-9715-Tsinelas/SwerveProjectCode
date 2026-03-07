package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterSubsystem extends SubsystemBase {
    // one motor reverse of the other
    // press y - > increase 10 percent
    // another button to cut power
    /*
     * 2 motor
     * can ids are not correct
     * 
     * to-do: add another motor config
     */

    
     private final SparkMax motorLeader = new SparkMax(10, MotorType.kBrushless);
     private final SparkMax motorFollower = new SparkMax(3, MotorType.kBrushless);
     private final SparkMax smallRoller = new SparkMax(21, MotorType.kBrushless);
     

    //private final SparkMax leader = new SparkMax(10, MotorType.kBrushless); // CHANGE to correct
    //private final SparkMax follower = new SparkMax(3, MotorType.kBrushless); // CHANGE to correct
    // other can id is 21 i think

    private double currentPower = 0.0;

    public ShooterSubsystem() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        // SparkMaxConfig leaderConfig = new SparkMaxConfig();
        // SparkMaxConfig followerConfig = new SparkMaxConfig();

        
        SparkMaxConfig motorFollowerConfig = new SparkMaxConfig();
        SparkMaxConfig smallRollerConfig = new SparkMaxConfig();
         

        baseConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kLargeMotorCurrentLimit)
            .inverted(true);

        
        motorLeader.configure(baseConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
         
        motorFollowerConfig.apply(baseConfig)
            .follow(motorLeader, true);
        motorFollower.configure(motorFollowerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        
        smallRollerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.kSmallMotorCurrentLimit)
            .inverted(true);

            
        smallRoller.configure(smallRollerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        

        // leaderConfig.apply(baseConfig).inverted(true);
        // leader.configure(leaderConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        // followerConfig.apply(baseConfig).inverted(false).follow(leader, false);
        // follower.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }
    
    public void setShooterPower(double power) {
        currentPower = Math.max(0.0, Math.min(1.0, power));
        // leader.set(currentPower);
        motorLeader.set(currentPower);
        smallRoller.set(currentPower);
    }

    public double getCurrentPower() {
        return currentPower;
    }

    public void stop() {
        setShooterPower(0.0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter/power", currentPower);
        //SmartDashboard.putNumber("Shooter/RPM", motorLeader.getEncoder().getVelocity());
        //SmartDashboard.putNumber("Shooter/Small motor temp", smallRoller.getMotorTemperature());
        //SmartDashboard.putNumber("Shooter/large motor temp", motorLeader.getMotorTemperature());
    }

}

