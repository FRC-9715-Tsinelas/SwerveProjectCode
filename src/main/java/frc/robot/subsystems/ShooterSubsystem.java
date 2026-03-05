package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;



public class ShooterSubsystem {
    // one motor reverse of the other
    // press y - > increase 10 percent
    // another button to cut power
    /*
     * 2 motor
     * can ids are not correct
     * 
     */

    private final SparkMax leader = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax follower = new SparkMax(6, MotorType.kBrushless);

    private double currentPower = 0.0;

    public ShooterSubsystem() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        baseConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50);

        leaderConfig.apply(baseConfig).inverted(true);
        leader.configure(leaderConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        followerConfig.apply(baseConfig).inverted(false).follow(leader, true);
        follower.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }
    
    public void setShooterPower(double power) {
        currentPower = Math.max(0.0, Math.min(1.0, power));
        leader.set(currentPower);
    }

    public double getCurrentPower() {
        return currentPower;
    }

    public void stop() {
        setShooterPower(0.0);
    }
}
