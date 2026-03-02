package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    public enum IntakeState {
        STOWED(IntakeConstants.kAngleStowed),
        GROUND(IntakeConstants.kAngleGround);

        public final double angleDegrees;

        IntakeState(double angle) { 
            this.angleDegrees = angle; 
        }
    }

    private final SparkMax roller;
    private final SparkMax pivot; 

    private final SparkAbsoluteEncoder pivotAbsEncoder;
    private final SparkClosedLoopController pivotController;
    
    private final ArmFeedforward feedforward;
    private IntakeState currentState = IntakeState.STOWED;

    public IntakeSubsystem() {
        roller = new SparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        pivot = new SparkMax(IntakeConstants.PIVOT_ID, MotorType.kBrushless);

        pivotAbsEncoder = pivot.getAbsoluteEncoder();
        pivotController = pivot.getClosedLoopController();

        feedforward = new ArmFeedforward(
            IntakeConstants.kS_PIVOT,
            IntakeConstants.kG_PIVOT, 
            IntakeConstants.kV_PIVOT
        );

        // roller config
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .smartCurrentLimit(IntakeConstants.kRollerCurrentLimit)
            .inverted(IntakeConstants.kRollerInverted);
        
        roller.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        // pivot config
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        
        pivotConfig
            .smartCurrentLimit(IntakeConstants.kPivotCurrentLimit) 
            .idleMode(IntakeConstants.kPivotIdleMode)
            .inverted(IntakeConstants.kPivotInverted);

        pivotConfig.absoluteEncoder
            .positionConversionFactor(360.0)
            .velocityConversionFactor(360.0 / 60.0)
            .zeroOffset(IntakeConstants.kEncoderOffset);
        
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(IntakeConstants.kP_PIVOT)
            .d(IntakeConstants.kD_PIVOT)
            .outputRange(-1.0, 1.0);

        
        pivot.configure(pivotConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        
    }

    public void setIntakeState(IntakeState state) {
        this.currentState = state;
    }

    public void setRollerOne(double speed) { 
        roller.set(speed); 
    }

    public void setRollerTwo(double speed) { 
        roller.set(speed); 
    }

    @Override
    public void periodic() {
        double currentAngleRad = Units.degreesToRadians(pivotAbsEncoder.getPosition());
        double ffVoltage = feedforward.calculate(currentAngleRad, 0);

        double targetAngle = Math.min(Math.max(currentState.angleDegrees, IntakeConstants.kMinAngle), IntakeConstants.kMaxAngle);
        pivotController.setSetpoint(
            targetAngle, 
            SparkMax.ControlType.kPosition, 
            com.revrobotics.spark.ClosedLoopSlot.kSlot0, 
            ffVoltage
        );
    }
}
