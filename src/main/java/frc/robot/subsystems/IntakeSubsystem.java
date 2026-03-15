package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;

import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase  {
    
    public enum IntakeState{
        STOWED(IntakeConstants.kAngleStowed),
        GROUND(IntakeConstants.kAngleGround);

        public final double angleDegrees;

        IntakeState(double angle) { this.angleDegrees = angle;}
    }

    private final SparkMax roller;
    private final SparkMax pivot;

    private final SparkAbsoluteEncoder pivotAbsoluteEncoder;
    private final SparkClosedLoopController pivotController;

    private final ArmFeedforward feedforward;
    private IntakeState currentState = IntakeState.STOWED;

    public double currentPower = 0.0;

    public IntakeSubsystem(){
        roller = new SparkMax(IntakeConstants.ROLLER_ID, MotorType.kBrushless);
        pivot = new SparkMax(IntakeConstants.PIVOT_ID, MotorType.kBrushless);

        pivotAbsoluteEncoder = pivot.getAbsoluteEncoder();
        pivotController = pivot.getClosedLoopController();

        feedforward = new ArmFeedforward(
            IntakeConstants.kS, 
            IntakeConstants.kG,
            IntakeConstants.kV 
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
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        //     .p(IntakeConstants.kP)
        //     .i(IntakeConstants.kI)
        //     .d(IntakeConstants.kD)
        //     .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);
        
        pivot.configure(pivotConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }
    

    public void setIntakeState(IntakeState state) {
        this.currentState = state;
    }

    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }

    public void runTestIntake(double speed) {
        //currentPower = Math.max(-1.0, Math.min(1.0, speed));
        pivot.set(speed);
    }

    public void setTargetAngle(double targetAngle) {
        double safeAngle = Math.min(Math.max(targetAngle, IntakeConstants.kMinAngle), IntakeConstants.kMaxAngle);
    }

    public void stop() {
        roller.set(0);
        pivot.set(0);
    }
    
    @Override
    public void periodic() {
        double currentAngle = pivotAbsoluteEncoder.getPosition();
        
        SmartDashboard.putNumber("Intake Arm Angle", currentAngle);
        //SmartDashboard.putNumber("Intake Arm Angle", Math.random());

        // double currentAngleRad = Units.degreesToRadians(pivotAbsoluteEncoder.getPosition());
        // double ffVoltage = feedforward.calculate(currentAngleRad, 0);

        // double targetAngle = Math.min(Math.max(currentState.angleDegrees, IntakeConstants.kMinAngle), IntakeConstants.kMaxAngle);
        // pivotController.setSetpoint(
        //     targetAngle, 
        //     SparkMax.ControlType.kPosition,
        //     com.revrobotics.spark.ClosedLoopSlot.kSlot0,
        //     ffVoltage
        // );
    }

    public Command runIntakeCommand(double speed) {
        return this.startEnd(
            () -> this.setRollerSpeed(speed),
            () -> this.stop()
        );
    }

}
