// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.ctre.phoenix6.swerve.SwerveRequest;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

// Path Planner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    // Subsystems Declaration
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem();
    private final IntakeSubsystem m_Intake = new IntakeSubsystem();
    private final IndexerSubsystem m_Indexer = new IndexerSubsystem();
    private final LEDSubsystem m_LED = new LEDSubsystem();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // List object for autos
    private final  SendableChooser<Command> autoChooser;
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private void configurePathPlanner() {
        // not working with try catch
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose, // reset,
                () -> drivetrain.getState().Speeds,

               (speeds, feedforwards) -> drivetrain.setControl(autoRequest.withSpeeds(speeds)),
               new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), //translation
                    new PIDConstants(5.0, 0.0, 0.0) //rotation
               ),
               config, // robot config
               () -> {
                    //should flip for alliance
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
               },
               drivetrain
            );
        } catch (Exception e) {
            DriverStation.reportError("can't find path planner config", e.getStackTrace());
        }
    }

    public RobotContainer() {
        configureBindings();

        // path planner
        configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //m_LED.setDefaultCommand(m_LED.run(() -> m_LED.setRed()));
        m_LED.setDefaultCommand(Commands.runOnce(m_LED::setRed, m_LED));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // pressing a brakes drivetrain like stop moving
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // it looks like pressing b points all the modules to wherever the left joystick is facing
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // SHOOTER commands
        // Up Key: Increase motor powers by 0.05 | Change when it's tuned
        joystick.povUp().onTrue(
            new InstantCommand(() -> {
                // System.out.println("up 1");
                // double nextLarge = m_Shooter.getLargePower() + 0.05;
                // double nextSmall = m_Shooter.getSmallPower() + 0.05;
                // System.out.println("up 2");
                // m_Shooter.setShooterPower(nextLarge, nextSmall); 
                m_Shooter.setShooterPower(0.55, 0.55); 
            }));
        // Down Key: Decrease motor powers by 0.05 | Change when it's tuned
        joystick.povDown().onTrue(
            new InstantCommand(() -> {
                System.out.println("down 1");
                double nextLarge = m_Shooter.getLargePower() + 0.05;
                double nextSmall = m_Shooter.getSmallPower() + 0.05;
                System.out.println("down 2");
                m_Shooter.setShooterPower(nextLarge, nextSmall); 
            }));
        // Left Key: Stop everything
        joystick.povLeft().onTrue(
            new InstantCommand(() -> {
                System.out.println("left pressed");
                m_Shooter.stop();
            }));

        //INTAKE commands -> Uncomment when tuned -> also set correct ids too
        //joystick.rightBumper().whileTrue(m_Intake.runIntakeCommand(2.0));

        joystick.rightBumper().onTrue(
            new InstantCommand(() -> {
                double nextSpeed = m_Intake.getCurrentPower() + 0.05;
                m_Intake.setRollerSpeed(nextSpeed);
            }));


        joystick.povRight().onTrue(
            new InstantCommand(() -> {
                System.out.println("right pressed");
                m_Intake.stop();
            }));
        
        // joystick.x().onTrue(
        //     new InstantCommand(() -> {
        //         m_Intake.runTestIntake(2);
        //     }));

        joystick.x().onTrue(
            m_Intake.startEnd(
                () -> m_Intake.runTestIntake(0.5),
                () -> m_Intake.stop()
            ).withTimeout(0.3));

        // INDEXER commands -> set correct IDs
        //joystick.y().onTrue(m_Indexer.toggleIndexer(1));
        joystick.y().whileTrue(
            m_Indexer.runEnd(
                () -> m_Indexer.setIndexerPower(0.6),
                () -> m_Indexer.stopIndexer()
            ));

        // joystick.y().onTrue(
        //     new InstantCommand(() -> {
        //         System.out.println("indexer up");
        //         double nextSpeed = m_Indexer.getCurrentPower() + 0.05;
        //         m_Indexer.setIndexerPower(nextSpeed); 
        //     }));

        // joystick.b().onTrue(
        //     new InstantCommand(() -> {
        //         // System.out.println("b pressed");
        //         // m_Indexer.stopIndexer();
        //         System.out.println("indexer down");
        //         double nextSpeed = m_Indexer.getCurrentPower() - 0.05;
        //         m_Indexer.setIndexerPower(nextSpeed); 
        //     }));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
    }
}
