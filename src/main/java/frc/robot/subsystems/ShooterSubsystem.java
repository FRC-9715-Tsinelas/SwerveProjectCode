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

     private final SparkMax motorLeader = new SparkMax(ShooterConstants.MOTOR_LEADER_ID, MotorType.kBrushless);
     private final SparkMax motorFollower = new SparkMax(ShooterConstants.MOTOR_FOLLOWER_ID, MotorType.kBrushless);
     private final SparkMax smallRoller = new SparkMax(ShooterConstants.SMALL_MOTOR_ID, MotorType.kBrushless);

    private double currentPower = 0.0;

    public ShooterSubsystem() {
        SparkMaxConfig baseConfig = new SparkMaxConfig();
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
    }
    
    public void setShooterPower(double power) {
        currentPower = Math.max(0.0, Math.min(1.0, power));
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

