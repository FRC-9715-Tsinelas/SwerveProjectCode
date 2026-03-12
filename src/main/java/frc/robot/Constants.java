package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {
    public static class IntakeConstants {
        // Motor IDs
        public static final int PIVOT_ID = 0; // Change to correct ID
        public static final int ROLLER_ID = 0; // Change to correct ID

        // General motor settings
        public static final boolean kRollerInverted = false; // Change if motor is moving wrong way
        public static final boolean kPivotInverted = false; // Change if motor is moving wrong way
        public static final IdleMode kPivotIdleMode = IdleMode.kBrake;
        public static final IdleMode kRollerIdleMode = IdleMode.kCoast;

        // current limits (amps)
        public static final int kPivotCurrentLimit = 40; // Adjust if needed
        public static final int kRollerCurrentLimit = 20;

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.0; // Voltage to overcome friction
        public static final double kV = 0.0; // to overcome gravity
        public static final double kG = 0.0; // velocity gain

        // constraints
        public static final double kMinOutput = -1.0;
        public static final double kMaxOutput = 1.0;

        // angles
        public static final double kAngleStowed = 0.0;
        public static final double kAngleGround = 90.0;

        // this gotta be adjusted by placing arm at 0 degrees then read the raw value
        public static final double kEncoderOffset = 0.0;

        //angles
        public static final double kMinAngle = 0.0;
        public static final double kMaxAngle = 90.0;
    }
    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 0; // change to correct port

        public static final boolean kIndexerInverted = false;

        public static final int kIndexerCurrentLimit = 25;
    }
    public static final class botaimConstants {
        public static final double kHubPoseX = 0; // need change
        public static final double kHubPoseY = 0; // need change
         = 
    }
}
