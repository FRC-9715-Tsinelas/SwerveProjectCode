package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {
    public static class IntakeConstants {
        // Motor IDs
        public static final int PIVOT_ID = 5; // Change to correct ID
        public static final int ROLLER_ID = 6; // Change to correct ID

        // General motor settings
        public static final boolean kRollerInverted = true; // Change if motor is moving wrong way
        public static final boolean kPivotInverted = false; // Change if motor is moving wrong way
        public static final IdleMode kPivotIdleMode = IdleMode.kBrake;
        public static final IdleMode kRollerIdleMode = IdleMode.kCoast;

        // current limits (amps)
        public static final int kPivotCurrentLimit = 45; // Adjust if needed
        public static final int kRollerCurrentLimit = 69;

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.0; // Voltage to overcome friction
        public static final double kV = 0.0; // vlocity gain -> 0.123
        public static final double kG = 0.5; // to overcome gravity

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
        public static final int INDEXER_MOTOR_ID = 4; // change to correct port

        public static final boolean kIndexerInverted = false;

        public static final int kIndexerCurrentLimit = 25;
    }
    public static final class ShooterConstants {

        public static final int MOTOR_LEADER_ID = 10;
        public static final int MOTOR_FOLLOWER_ID = 3;
        public static final int SMALL_MOTOR_ID = 21;

        // current limits
        public static final int kLargeMotorCurrentLimit = 35;
        public static final int kSmallMotorCurrentLimit = 20;
    }
}
