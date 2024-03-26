package frc.system;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.*;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.config.SwerveConfig;
import frc.system.vision.Vision;
import frc.system.vision.Vision.Measurement;

public class Swerve extends SwerveDrivetrain implements LoggedSubsystems, Consumer<Vision.Measurement> {
    private final String simpleName = this.getClass().getSimpleName();

    // Control Modes
    private final SwerveDriveBrake cachedBrake = new SwerveDriveBrake();
    private final FieldCentric cachedFieldCentric = new FieldCentric();
    private final FieldCentricFacingAngle cachedFieldCentricFacing = new FieldCentricFacingAngle();
    private final ApplyChassisSpeeds AutoRequest = new ApplyChassisSpeeds();

    // Network
    private final NetworkTable Table;
    private final StructPublisher<Pose2d> ntPose2d;
    private final StructPublisher<Pose3d> ntPose3d;
    private final StructArrayPublisher<SwerveModuleState> ntSwerveModuleState;
    private final StructArrayPublisher<SwerveModuleState> ntSwerveModuleTarget;

    // vars
    private final PathConstraints ppConstraints;
    private final Rotation2d autoAimDeadband = Rotation2d.fromDegrees(5);

    private boolean hasPose = false;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final AtomicReference<SwerveDriveState> state = new AtomicReference<>();

    /**
     * The rotation override for the drivetrain. Set to null if there is no
     * override.
     */
    private Rotation2d rotationOverride;

    public Swerve(
            PathConstraints constraints,
            double kSpeedAt12VoltsMps,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        this.Table = NetworkTableInstance.getDefault().getTable("Subsystems").getSubTable(simpleName);

        zeroGyro();

        // Autos
        this.ppConstraints = constraints;
        configurePathPlanner();

        // Logging
        ntPose2d = Table.getStructTopic("Pose2d", new Pose2dStruct()).publish();
        ntPose3d = Table.getStructTopic("Pose3d", new Pose3dStruct()).publish();
        ntSwerveModuleState = Table.getStructArrayTopic("SwerveModuleState", SwerveModuleState.struct).publish();
        ntSwerveModuleTarget = Table.getStructArrayTopic("SwerveModuleTarget", SwerveModuleState.struct).publish();

        registerTelemetry((s) -> {
            ntPose2d.set(s.Pose);
            state.set(s);
            ntPose3d.set(new Pose3d(s.Pose));
            ntSwerveModuleState.set(s.ModuleStates);
            ntSwerveModuleTarget.set(s.ModuleTargets);
        });

        if (Utils.isSimulation()) {
            startSimThread();
        }

        register();
    }

    // ---------- Generic Control ----------

    private Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Pose2d pose() {
        return state.get().Pose;
    }

    public boolean hasPose() {
        return hasPose;
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /** Returns whether the drivetrain is facing the stored rotational override. */
    public boolean isAimed() {
        if (rotationOverride == null) {
            return false;
        }
        double rotationError = Math.abs(pose().getRotation().minus(rotationOverride).getRadians());
        // TODO: check logic worried about bounds being [-pi,pi]
        return rotationError <= autoAimDeadband.getRadians();
    }

    /**
     * Sets a rotational override. Provide a value of `null` to clear the override.
     */
    public void setRotationOverride(Rotation2d rotation) {
        rotationOverride = rotation;
    }

    private Optional<Rotation2d> getRotationTargetOverride() {
        return Optional.ofNullable(rotationOverride);
    }

    // ---------- PathPlanner/Auto ----------
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                this::pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        SwerveConfig.kSpeedAt12VoltsMps,
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

    public void addPPAutos(SendableChooser<Command> autos) {
        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autos.addOption(autoName + " PP", AutoBuilder.buildAuto(autoName));
        }
    }

    // ---------- Commands ----------

    public Command zeroGyro() {
        return runOnce(() -> {
            hasPose = false;
            seedFieldRelative(
                    new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(218), Rotation2d.fromDegrees(0)));
        });

        // return runOnce(() -> {
        // hasPose = false;
        // seedFieldRelative(
        // new Pose2d(Units.inchesToMeters(600), Units.inchesToMeters(218),
        // Rotation2d.fromDegrees(270)));
        // });
    }

    public Command driveTo(Pose2d target, double velocity) {
        return AutoBuilder.pathfindToPose(target, ppConstraints, velocity);
    }

    public Command DriveToThenPath(PathPlannerPath path) {

        return AutoBuilder.pathfindThenFollowPath(path, ppConstraints, 0);
    }

    /**
     * Values are squared!!!! this adds more perdition at low speeds
     *
     * @param xVel [-1,1] percent speed + is forward
     * @param yVel [-1,1] percent speed + is Left
     * @param rVel [-1, 1] percent angler rate + CC+ // TODO: check this value
     * @return
     */
    public Command drive(DoubleSupplier xVel, DoubleSupplier yVel, DoubleSupplier rVel) {
        return applyRequest(() -> {
            double x = xVel.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;
            double y = yVel.getAsDouble() * SwerveConfig.kSpeedAt12VoltsMps;

            if (rotationOverride == null)
                return cachedFieldCentric.withVelocityX(x).withVelocityY(y)
                        .withRotationalRate(rVel.getAsDouble() * SwerveConfig.kMaxAngularRate);
            else
                return cachedFieldCentricFacing.withVelocityX(x).withVelocityY(y).withTargetDirection(rotationOverride);
        });
    }

    public Command brake() {
        return applyRequest(
                () -> cachedBrake);
    }

    // ---------- Vision ----------
    public void accept(Measurement t) {
        if (hasPose) {                                              // Just double checking is this just so that the first time we run this we set seedFieldRelative?
                                                                    // we absolutely need to confirm that we're within 1 meter of where we think we are before we add these values to the filter...
            if (t.stdDev() != null) {
                addVisionMeasurement(t.pose().toPose2d(), t.timestamp(), t.stdDev());               // Let's make sure this isn't null.
                                                                                                    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html#addVisionMeasurement(edu.wpi.first.math.geometry.Pose2d,double) This says that we need to dump those values and this is for teams that aren't running as crazy high FPS as we are...
            } else {
                addVisionMeasurement(t.pose().toPose2d(), t.timestamp());
            }
        } else {
            seedFieldRelative(t.pose().toPose2d());
            hasPose = true;
        }
    }

    // ---------- Sim ----------
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

    // ---------- Logging ----------
    @Override
    public void log() {
        //TODO: add motor data

    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
}
