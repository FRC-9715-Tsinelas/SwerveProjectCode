package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax indexerMotor;
    private final SparkMaxConfig indexerConfig;

    private boolean isRunning = false;

    public IndexerSubsystem() {
        // changed brushless to brushed
        indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushed);
        indexerConfig = new SparkMaxConfig();

        indexerConfig
            .idleMode(IdleMode.kBrake)
            .inverted(IndexerConstants.kIndexerInverted)
            .smartCurrentLimit(IndexerConstants.kIndexerCurrentLimit);

        indexerMotor.configure(
            indexerConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );
    }

    public void runIndexer(double speed) {
        indexerMotor.set(speed);
        isRunning = true;
    }

    public void stopIndexer() {
        indexerMotor.set(0);
        isRunning = false;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public Command toggleIndexer(double speed) {
        return runOnce(() -> {
            if (isRunning) {
                stopIndexer();
            } else {
                runIndexer(speed);
            }
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Indexer Running", isRunning);
        SmartDashboard.putNumber("Indexer Output", indexerMotor.getAppliedOutput());
    }
}