// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

// pathplanner
import com.pathplanner.lib.auto.NamedCommands;

// Logging
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.cmd.AimSpeaker;
import frc.config.SwerveConfig;
import frc.system.*;

public final class Main extends TimedRobot {
    public static void main(String... args) {
        RobotBase.startRobot(Main::new);
    }

    private SendableChooser<Command> autonomousChooser = new SendableChooser<>();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final NetworkTable subsystemsTable = NetworkTableInstance.getDefault().getTable("Subsystems");

    private final Swerve drivetrain = SwerveConfig.swerve;

    private final Transit transit = new Transit(subsystemsTable);
    private final Intake intake = new Intake(subsystemsTable);
    private final Shooter shooter = new Shooter(subsystemsTable);
    private final Pivot pivot = new Pivot(subsystemsTable);

    // Utilities
    static double squareInput(double input) {
        return Math.copySign(input * input, input);
    }

    // TODO: add comments
    private class Commands {
        Command rumble(RumbleType type, double strength, double duration,
                CommandXboxController... controllers) {
            return new Command() {
                @Override
                public void initialize() {
                    for (CommandXboxController controller : controllers)
                        controller.getHID().setRumble(type, strength);
                }

                @Override
                public void end(boolean interrupted) {
                    for (CommandXboxController controller : controllers)
                        controller.getHID().setRumble(type, 0);
                }
            }.withTimeout(duration);
        }

        Command intakeNote() {
            return pivot.intake().andThen(transit.feedIn().deadlineWith(intake.runIn()));
        }

        Command primeAmp() {
            return pivot.amp().alongWith(shooter.shootAmp());
        }

        Command primeSpeaker() {
            return new AimSpeaker(drivetrain, pivot).alongWith(shooter.shootSpeaker());
        }

        Command primeSubwoofer() {
            return pivot.subwoofer().alongWith(shooter.shootSpeaker());
        }

        Command waitThenFeed() {
            return shooter.waitPrimed().andThen(transit.feedOut());
            // TODO: check if pivot is at angle
        }
    }

    private final Commands cmds = new Commands();

    private void configButtonBindings() {
        configNamedCommands();
        // ----------- DEFAULT COMMANDS -----------

        transit.hasNoteTrigger.onTrue(cmds.rumble(RumbleType.kBothRumble, 0.5, 0.25));
        transit.hasNoteTrigger.onFalse(cmds.rumble(RumbleType.kBothRumble, 0.5, 0.25));

        // NOTE: it appears that default commands are immediately rescheduled if they
        // finish. Looks like we'll have to implement some special logic to go to the
        // intake by default.
        // pivot.setDefaultCommand(pivot.toIntake());
        pivot.setDefaultCommand(pivot.velocity(operator::getLeftY));

        // ----------- DRIVER CONTROLS -----------

        drivetrain.setDefaultCommand(drivetrain.drive(
                () -> squareInput(driver.getLeftY()),
                () -> squareInput(driver.getLeftX()),
                // TODO: ask if driver wants turning squared as well
                () -> -driver.getRightX()));

        driver.leftBumper().onTrue(drivetrain.zeroGyro());
        driver.x().whileTrue(drivetrain.brake());
        driver.leftTrigger().onTrue(cmds.intakeNote());
        driver.leftTrigger().onTrue(new InstantCommand(() -> {
            System.out.println("Intakeing Note");
        }));

        // ---------- OPERATOR CONTROLS ----------

        // TODO: Pathfind to the amp using a PathfindToPose command
        operator.leftBumper().whileTrue(cmds.primeAmp());
        operator.leftTrigger().whileTrue(cmds.primeSpeaker());
        operator.rightTrigger().whileTrue(transit.runForwards());

        operator.b().whileTrue(transit.runBackward());
        operator.x().whileTrue(intake.runOut());
        operator.y().whileTrue(cmds.primeSubwoofer());
        // Zeroes the pivot, assuming it is at intaking position.
        operator.start().onTrue(new InstantCommand(pivot::zeroToIntake));

        // TODO: Add climb command
    }

    private void configAutos() {
        configNamedCommands();
        // ------------------ AUTOS ------------------
        // Adds pathplaner paths. TODO: fix crash here
        // autonomousChooser = drivetrain.getAutoPaths();

        autonomousChooser = new SendableChooser<>();

        // TODO: Ensure a default `null` command is available, and remove this option.
        autonomousChooser.setDefaultOption("Nothing", new Command() {
        });

        // TODO: check speed of back-out
        autonomousChooser.addOption("Back-out (NO PP)", drivetrain.drive(() -> -0.5, () -> 0, () -> 0));

        autonomousChooser.addOption("Shoot",
                cmds.primeSpeaker().raceWith(
                        new WaitUntilCommand(() -> drivetrain.isAimed() && pivot.isAimed())
                                .andThen(cmds.waitThenFeed()))
                        // TODO: configure the next two as default commands (not working)
                        .andThen(shooter.stop().alongWith(pivot.intake())));

        autonomousChooser.addOption("Shoot against subwoofer",
                new WaitCommand(5).andThen(cmds.primeSubwoofer().raceWith(cmds.waitThenFeed())));

        drivetrain.addPPAutos(autonomousChooser);

        SmartDashboard.putData("auto", autonomousChooser);
    }

    private void configNamedCommands() {
        NamedCommands.registerCommand("Intake", cmds.intakeNote());
        NamedCommands.registerCommand("Shoot", cmds.waitThenFeed());
        NamedCommands.registerCommand("PrimeAmp", cmds.primeAmp());
        NamedCommands.registerCommand("PrimeSpeaker", cmds.primeSpeaker());
    }

    @Override
    public void robotInit() {
        // ------------------- Logging -------------------
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        // TODO: re-enable vision once the jitter is solved.
        new Nt().register(drivetrain);

        configAutos();
        configButtonBindings();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    private Command autonomousCommand;

    @Override
    public void autonomousInit() {
        autonomousCommand = autonomousChooser.getSelected();
        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        // TODO: Run an automated full systems check
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
