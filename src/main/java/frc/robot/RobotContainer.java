// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.*;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = TunerConstants.MaxAngularRate; // 3/4 of a rotation per second max angular velocity
    private Pose2d speakerPose = new Pose2d();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver = new CommandXboxController(0); // My joystick
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystem
    private final DrivetrainSub drivetrain = TunerConstants.DriveTrain;
    private FeederSub feeder = new FeederSub();
    private ShooterSub shooter = new ShooterSub();
    private PivotSub pivot = new PivotSub();
    private IntakeSub intake = new IntakeSub(() -> feeder.hasNote());

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Path follower */
    private final SendableChooser<Command> autoChooser = drivetrain.getAutoPaths();

    public RobotContainer() {
        configureBindings();
        configureNamedCommands();

        // new Vision(5892, drivetrain).start();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Subsystems
        SmartDashboard.putData("Intake", intake);
        SmartDashboard.putData("Intake", feeder);
        SmartDashboard.putData("Intake", shooter);
        SmartDashboard.putData("Intake", pivot);

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    private void configureBindings() {
        // Auto
        autoChooser.addOption("CmdBased-1note", pivot.speekerPose(drivetrain.getState().Pose).withTimeout(1)
                .andThen(shooter.shoot(true).alongWith(new WaitCommand(1).andThen(feeder.feed(true))).withTimeout(3)));

        // Driver
        drivetrain.setDefaultCommand(drivetrain.drive(
                -driver.getLeftY() * MaxSpeed,
                -driver.getLeftX() * MaxSpeed,
                -driver.getRightX() * MaxAngularRate));

        driver.leftBumper().onTrue(
                drivetrain.zeroGyro());

        driver.x().whileTrue(
                drivetrain.xState());

        driver.rightTrigger().whileTrue(
                feeder.feed(true).alongWith(intake.intake(true).onlyIf(pivot::atIntakePos)));

        // if (Utils.isSimulation()) {
        // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
        // Rotation2d.fromDegrees(90)));
        // }

        // operator
        pivot.setDefaultCommand(pivot.intakePos());

        operator.leftTrigger().whileTrue(
                shooter.shoot(true));
        operator.leftTrigger().whileTrue(
                drivetrain.driveAt(
                        -driver.getLeftY() * MaxSpeed,
                        -driver.getLeftX() * MaxSpeed,
                        speakerPose.minus(drivetrain.getState().Pose).getRotation()));
        operator.leftTrigger().whileTrue(
                pivot.speekerPose(drivetrain.getState().Pose));

        operator.rightTrigger().whileTrue(
                feeder.feed(true));

        operator.a().whileTrue(pivot.ampPose());
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("IntakePos", pivot.intakePos());
        NamedCommands.registerCommand("SeekerPose", pivot.speekerPose(speakerPose));
        NamedCommands.registerCommand("AmpPose", pivot.ampPose());
        NamedCommands.registerCommand("Intake", intake.intake(true).withTimeout(1));
        NamedCommands.registerCommand("Shoot",
                shooter.shoot(true).alongWith(new WaitCommand(1).andThen(feeder.feed(true))).withTimeout(3));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
