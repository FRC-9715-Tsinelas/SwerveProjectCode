package frc.robot.subsystems
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public void autoaim(DoubleSupplier xSup, DoubleSupplier ySup) {
    Pose2d currentPose = m_drivetrain.getState().Pose;

    double deltaX = botaimConstants.kHubPoseX - currentPose.getX();
    double deltaY = botaimConstants.kHubPoseY - currentPose.getY();
    double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

    m_drivetrain.setControl(
        new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(xSup.getAsDouble() * MaxSpeed) // Move X
            .withVelocityY(ySup.getAsDouble() * MaxSpeed) // Move Y
            .withTargetDirection(Rotation2d.fromDegrees(targetAngle)) // Aim
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    );
}
