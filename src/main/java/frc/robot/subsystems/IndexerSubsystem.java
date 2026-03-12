package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    // private final SparkMax indexerMotor;
    // private final SparkMaxConfig indexerConfig;

    // private boolean isRunning = false;

    // public IndexerSubsystem() {
        // changed brushless to brushed
    //     indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushed);
    //     indexerConfig = new SparkMaxConfig();

    //     indexerConfig
    //         .idleMode(IdleMode.kBrake)
    //         .inverted(IndexerConstants.kIndexerInverted)
    //         .smartCurrentLimit(IndexerConstants.kIndexerCurrentLimit);

    //     indexerMotor.configure(
    //         indexerConfig,
    //         com.revrobotics.ResetMode.kResetSafeParameters,
    //         com.revrobotics.PersistMode.kPersistParameters
    //     );
    // }

    // public void runIndexer(double speed) {
    //     indexerMotor.set(speed);
    //     isRunning = true;
    // }

    // public void stopIndexer() {
    //     indexerMotor.set(0);
    //     isRunning = false;
    // }

    // public boolean isRunning() {
    //     return isRunning;
    // }

    // public Command toggleIndexer(double speed) {
    //     return runOnce(() -> {
    //         if (isRunning) {
    //             stopIndexer();
    //         } else {
    //             runIndexer(speed);
    //         }
    //     });
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Indexer Running", isRunning);
        // SmartDashboard.putNumber("Indexer Output", indexerMotor.getAppliedOutput());
    }
}

//public class IndexerSubsystem extends SubsystemBase {
    // private final SparkMax indexer = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    // private final SparkMaxConfig indexerConfig = new SparkMaxConfig();

    // private boolean indexerRunning = false;

    // public IndexerSubsystem() {

    //     indexerConfig
    //         .idleMode(IdleMode.kBrake) 
    //         .inverted(IndexerConstants.kIndexerInverted)
    //         .smartCurrentLimit(IndexerConstants.kIndexerCurrentLimit);

    //     indexer.configure(indexerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    // }

    // public void runIndexer(double speed) {
    //     indexer.set(speed);
    //     indexerRunning = true;
    // }

    // public void stopIndexer() {
    //     indexer.set(0);
    //     indexerRunning = false;
    // }

    // public Command toggleIndexer(double speed) {
    //     return runOnce(() -> {
    //         if (indexerRunning) {
    //             stopIndexer();
    //         } else {
    //             runIndexer(speed);
    //         }
    //     });
    // }
//} 

