package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {
    public static final class IntakeConstants {
        // Motor ID
        public static final int PIVOT_ID = 1; // CHANGE
        public static final int ROLLER_MOTOR_ID = 2; // CHANGE

        // settings
        public static final boolean kPivotInverted = false;
        public static final boolean kRollerInverted = false;
        public static final IdleMode kPivotIdleMode = IdleMode.kBrake; 
        public static final IdleMode kRollerIdleMode = IdleMode.kCoast;

        // current limits (amps)
        public static final int kPivotCurrentLimit = 40; 
        public static final int kRollerCurrentLimit = 20;

        // PID
        public static final double kP_PIVOT = 0.0; 
        public static final double kI_PIVOT = 0.0;
        public static final double kD_PIVOT = 0.0;

        // feedforward
        public static final double kS_PIVOT = 0.1; // Voltage to overcome static friction
        public static final double kG_PIVOT = 0.4; // Voltage to overcome gravity
        public static final double kV_PIVOT = 0.0; // Velocity gain

        // motion constraints
        public static final double kMinOutput = -1.0;
        public static final double kMaxOutput = 1.0;

        // absolute encoder setup
        // find this by placing arm at 0 degrees and reading value
        public static final double kEncoderOffset = 0.0; 

        // arm set
        public static final double kAngleStowed = 0.0;   // pushed up
        public static final double kAngleGround = 90.0; // on floor

        // ramp rates
        public static final double kOpenLoopRampRate = 0.5; 
        public static final double kClosedLoopRampRate = 0.0;

        // angles
        public static final double kMinAngle = 0.0;
        public static final double kMaxAngle = 90.0;
  }
}
