package frc.robot.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class DrivetrainSub extends SwerveDrivetrain implements SubsystemReq {
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public DrivetrainSub(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public DrivetrainSub(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        this.seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }

    }

    public SendableChooser<Command> getAutoPaths() {
        return AutoBuilder.buildAutoChooser();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1)
            .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveAt = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1)
            .withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake xState = new SwerveRequest.SwerveDriveBrake();

    public Command drive(double VelocityX, double VelocityY, double RotationalRate) {
        return applyRequest(() -> drive
                .withVelocityX(VelocityX)
                .withVelocityY(VelocityY)
                .withRotationalRate(RotationalRate));
    }

    public Command driveAt(double VelocityX, double VelocityY, Rotation2d TargetDirection) {
        return applyRequest(() -> driveAt
                .withVelocityX(VelocityX)
                .withVelocityY(VelocityY)
                .withTargetDirection(TargetDirection));
    }

    public Command xState() {
        return applyRequest(() -> xState);
    }

    public Command zeroGyro() {
        return runOnce(() -> this.seedFieldRelative());
    }

    @Override
    public void disable() {
        applyRequest(() -> xState);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        // // Some condition that should decide if we want to override rotation
        // if (Limelight.hasGamePieceTarget()) {
        // // Return an optional containing the rotation override (this should be a
        // field
        // // relative rotation)
        // return Optional.of(Limelight.getRobotToGamePieceRotation());
        // } else {
        // // return an empty optional when we don't want to override the path's
        // rotation
        return Optional.empty();
        // }
    }

    public Pose2d getPose() {
        return getState().Pose;
    }
}
