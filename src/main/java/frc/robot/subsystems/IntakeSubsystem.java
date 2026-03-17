package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
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

    //private final SparkAbsoluteEncoder pivotAbsoluteEncoder;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    private ArmFeedforward feedforward; // REMOVED final bring back once it's done
    private IntakeState currentState = IntakeState.STOWED;

    public double currentPower = 0.0;

    double p = 0.0;
    double g = 0.0;

    // pivot config
    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    public IntakeSubsystem(){
        roller = new SparkMax(IntakeConstants.ROLLER_ID, MotorType.kBrushless);
        pivot = new SparkMax(IntakeConstants.PIVOT_ID, MotorType.kBrushless);

        //pivotAbsoluteEncoder = pivot.getAbsoluteEncoder();
        pivotController = pivot.getClosedLoopController();
        pivotEncoder = pivot.getEncoder();

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
        
        

        pivotConfig
            .smartCurrentLimit(IntakeConstants.kPivotCurrentLimit)
            .idleMode(IntakeConstants.kPivotIdleMode)
            .inverted(IntakeConstants.kPivotInverted);

        // pivotConfig.absoluteEncoder
        //     .positionConversionFactor(360.0)
        //     .velocityConversionFactor(360.0 / 60.0)
        //     .zeroOffset(IntakeConstants.kEncoderOffset);

        pivotConfig.encoder
            .positionConversionFactor(360.0 / 12.5)
            .velocityConversionFactor(360.0 / 12.5 / 60);

        pivotConfig.softLimit
            .forwardSoftLimit(95)
            .reverseSoftLimit(-5.0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);
        
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // pivotConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        //     .p(IntakeConstants.kP)
        //     .i(IntakeConstants.kI)
        //     .d(IntakeConstants.kD)
        //     .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);
        
        pivot.configure(pivotConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        pivotEncoder.setPosition(95);

        SmartDashboard.putNumber("P Value", p);
        SmartDashboard.putNumber("G Value", g);
    }

    public double getCurrentPower() {
        return currentPower;
    }
    
    public void setIntakeState(IntakeState state) {
        this.currentState = state;
    }

    public void setRollerSpeed(double speed) {
        currentPower = Math.max(-1.0, Math.min(1.0, speed));
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
        currentPower = 0;
    }

    public void setAngle(double targetAngle) {
        
        double ffVoltage = feedforward.calculate(Math.toRadians(pivotEncoder.getPosition()), 0);
        System.out.println("set angle pressed");
        pivotController.setSetpoint(
            targetAngle,
            SparkMax.ControlType.kPosition,
            com.revrobotics.spark.ClosedLoopSlot.kSlot0,
            ffVoltage
        );
        System.out.println("applied");
    }

    
    
    @Override
    public void periodic() {
        double currentAngle = pivotEncoder.getPosition();
        SmartDashboard.putNumber("Intake Arm Angle", currentAngle);

        SmartDashboard.putNumber("Intake Roller Speed", currentPower);

        // double testkG = 0.25;

        // double currentAngleRad = Math.toRadians(pivotEncoder.getPosition());

        // double outputVoltage = testkG * Math.cos(currentAngleRad);
        // pivot.setVoltage(outputVoltage);

        // p = SmartDashboard.getNumber("P Value", p);
        // g = SmartDashboard.getNumber("G Value", g);

        // double ffVoltage = feedforward.calculate(Math.toRadians(pivotEncoder.getPosition()), 0);
        // pivotConfig.closedLoop.p(p);
        // pivot.configure(pivotConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        // this.feedforward = new ArmFeedforward(0, g, 0);

        // pivotController.setSetpoint(
        //     0.0,
        //     SparkMax.ControlType.kPosition,
        //     com.revrobotics.spark.ClosedLoopSlot.kSlot0,
        //     ffVoltage
        // );
        
        // double newP = SmartDashboard.getNumber("P Value", p);
        // double newG = SmartDashboard.getNumber("G Value", g);

        
        // if (newP != p || newG != g) {
        //     p = newP;
        //     g = newG;

            
        //     pivotConfig.closedLoop.p(p);
        //     pivot.configure(pivotConfig, 
        //         com.revrobotics.ResetMode.kNoResetSafeParameters, 
        //         com.revrobotics.PersistMode.kNoPersistParameters);

        //     // update
        //     this.feedforward = new ArmFeedforward(IntakeConstants.kS, g, IntakeConstants.kV);
        // }

        // //
        // double currentPosRad = Math.toRadians(pivotEncoder.getPosition());
        // //currentPosRad = currentPosRad * -1;
        // double ffVoltage = feedforward.calculate(currentPosRad, 0);

        // pivotController.setSetpoint(
        //     0.0, 
        //     SparkMax.ControlType.kPosition,
        //     com.revrobotics.spark.ClosedLoopSlot.kSlot0,
        //     ffVoltage
        // );

        // SmartDashboard.putNumber("Intake Arm Angle", pivotAbsoluteEncoder.getPosition());
        // SmartDashboard.putNumber("Intake velocity", pivotAbsoluteEncoder.getVelocity());

        double currentAngleRad = Units.degreesToRadians(pivotEncoder.getPosition());
        double ffVoltage = feedforward.calculate(currentAngleRad, 0);

        double targetAngle = Math.min(Math.max(currentState.angleDegrees, IntakeConstants.kMinAngle), IntakeConstants.kMaxAngle);
        // pivotController.setSetpoint(
        //     0, 
        //     SparkMax.ControlType.kPosition,
        //     com.revrobotics.spark.ClosedLoopSlot.kSlot0,
        //     ffVoltage
        // );
        //pivot.setVoltage(ffVoltage);
    }

    public Command runIntakeCommand(double speed) {
        return this.startEnd(
            () -> this.setRollerSpeed(speed),
            () -> this.stop()
        );
    }

}
