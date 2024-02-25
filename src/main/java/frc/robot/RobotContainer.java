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

    // Subsytems
    private final DrivetrainSub drivetrain = TunerConstants.DriveTrain;
    private IntakeSub intake = new IntakeSub();
    private FeederSub feeder = new FeederSub();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Path follower */
    private final SendableChooser<Command> autoChooser = drivetrain.getAutoPaths();

    public RobotContainer() {
        // new Vision(5892, drivetrain).start();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Subsystems
        SmartDashboard.putData("Intake", intake);

        drivetrain.registerTelemetry(logger::telemeterize);

        configureBindings();
        configureNamedCommands();
    }

    private void configureBindings() {
        // Auto
        autoChooser.addOption("Cmdbased-1note", mec.pivot(0.028564453125).withTimeout(1)
                .andThen(mec.shooter(true).alongWith(new WaitCommand(1).andThen(feeder.feed(true))).withTimeout(3)));

        // Driver
        drivetrain.setDefaultCommand(drivetrain.drive(
                -driver.getLeftY() * MaxSpeed,
                -driver.getLeftX() * MaxSpeed,
                -driver.getRightX() * MaxAngularRate));

        driver.a().whileTrue(drivetrain.driveAt(
                -driver.getLeftY() * MaxSpeed,
                -driver.getLeftX() * MaxSpeed,
                speakerPose.minus(drivetrain.getState().Pose).getRotation()));

        driver.b().whileTrue(drivetrain.xState());

        driver.leftBumper().onTrue(drivetrain.zeroGyro());

        // if (Utils.isSimulation()) {
        // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
        // Rotation2d.fromDegrees(90)));
        // }

        // operator
        // piviot.setDefultCommand(intapePose)
        operator.leftTrigger().whileTrue(mec.shooter(true));
        // operator.rightTrigger().whileTrue(shooter(-1)).onFalse(shooter(0));

        operator.rightTrigger().whileTrue(feeder.feed(true));
        operator.a().whileTrue(feeder.feed(true).alongWith(intake.intake(true)));
        operator.b().whileTrue(feeder.feed(false));
        operator.y().whileTrue(intake.intake(true));

        operator.povDown().whileTrue(mec.pivot(0.01));
        operator.povLeft().whileTrue(mec.pivot(0.028564453125));
        operator.povRight().whileTrue(mec.pivot(0.310302734375));
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("PivIntakePos", mec.pivot(0.01));
        NamedCommands.registerCommand("PivShootPos", mec.pivot(0.028564453125));
        NamedCommands.registerCommand("PivShootPos2", mec.pivot(0.310302734375));
        NamedCommands.registerCommand("Intake", intake.intake(true).withTimeout(1));
        NamedCommands.registerCommand("Shoot", mec.shooter(true).alongWith(feeder.feed(true)).withTimeout(1));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
