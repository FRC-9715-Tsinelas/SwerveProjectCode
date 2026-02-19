package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {
    public static class IntakeConstants {
        // Motor IDs
        public static final int kRollerMotorId = 0; // Change to correct ID
        public static final int kPivotMotorId = 0; // Change to correct ID

        // General motor settings
        public static final boolean kRollerInverted = false; // Change if motor is moving wrong way
        public static final boolean kPivotInverted = false; // Change if motor is moving wrong way

        public static final int kCurrentLimit = 40; // Adjust if needed
        public static final IdleMode kIdleMode = IdleMode.kCoast; // Controls behavior when idle

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kG = 0.0;
        public static final double kA = 0.0;

        public static final double TOLERANCE = 0.05; // Allowed error
    }
}
