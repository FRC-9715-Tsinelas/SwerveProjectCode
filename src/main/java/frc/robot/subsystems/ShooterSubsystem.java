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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    /*
     * private final SparkMax motorLeader = new SparkMax(10, MotorType.kBrushless);
     * private final SparkMax motorFollower = new SparkMax(3, MotorType.kBrushless);
     * private final SparkMax smallRoller = new SparkMax(21, MotorType.kBrushless);
     */

    private final SparkMax leader = new SparkMax(10, MotorType.kBrushless); // CHANGE to correct
    private final SparkMax follower = new SparkMax(3, MotorType.kBrushless); // CHANGE to correct
    // other can id is 21 i think

    private double currentPower = 0.0;

    public ShooterSubsystem() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        /*
         * SparkMaxConfig motorFollowerConfig = new SparkMaxConfig();
         * SparkMaxConfig smallRollerConfig = new SparkMaxConfig();
         */

        baseConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50);

        /*
         * motorLeader.configure(baseConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
         * 
         * motorFollowerConfig.apply(baseConfig)
         *   .follow(motorLeader, false);
         * motorFollower.configure(motorFollowerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
         * 
         * smallRollerConfig.apply(baseConfig);
         * smallRoller.configure(baseConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
         * 
         */

        leaderConfig.apply(baseConfig).inverted(true);
        leader.configure(leaderConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        followerConfig.apply(baseConfig).inverted(false).follow(leader, true);
        follower.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }
    
    public void setShooterPower(double power) {
        currentPower = Math.max(0.0, Math.min(1.0, power));
        leader.set(currentPower);
        // motorLeader.set(currentPower);
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
