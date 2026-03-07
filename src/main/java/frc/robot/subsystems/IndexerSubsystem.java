package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexerSubsystem extends SubsystemBase {
    private final SparkMax indexer = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig indexerConfig = new SparkMaxConfig();

    private boolean indexerRunning = false;

    public IndexerSubsystem() {

        indexerConfig
            .idleMode(IdleMode.kBrake) 
            .inverted(IndexerConstants.kIndexerInverted)
            .smartCurrentLimit(IndexerConstants.kIndexerCurrentLimit);

        indexer.configure(indexerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    public void runIndexer(double speed) {
        indexer.set(speed);
        indexerRunning = true;
    }

    public void stopIndexer() {
        indexer.set(0);
        indexerRunning = false;
    }

    public Command toggleIndexer(double speed) {
        return runOnce(() -> {
            if (indexerRunning) {
                stopIndexer();
            } else {
                runIndexer(speed);
            }
        });
    }
} 

