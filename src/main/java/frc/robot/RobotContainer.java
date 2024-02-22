// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps / 2; // kSpeedAt12VoltsMps desired top speed
	private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

	private Pose2d speakerPose = new Pose2d();

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController driver = new CommandXboxController(0); // My joystick
	private final CommandXboxController operator = new CommandXboxController(1);

	private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
	// // My drivetrain

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

	private final SwerveRequest.FieldCentricFacingAngle driveAt = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(MaxSpeed);

	/* Path follower */
	private Command runAuto = drivetrain.getAutoPath("Tests");

	private void configureBindings() {
		new Mechanism(driver, operator);
		new Vision(5892, drivetrain);

		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
																									// negative Y
																									// (forward)
						.withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
																					// negative X (left)
				));

		driver.rightBumper().onTrue(drivetrain.applyRequest(() -> driveAt
				.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
				// negative Y
				// (forward)
				.withVelocityY(-driver.getLeftX() * MaxSpeed)
				.withTargetDirection(speakerPose.minus(drivetrain.getState().Pose).getRotation())));

		driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
		driver.b().whileTrue(drivetrain
				.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

		// reset the field-centric heading on left bumper press
		driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
					Rotation2d.fromDegrees(90)));
		}
		drivetrain.registerTelemetry(logger::telemeterize);
	}

	public RobotContainer() {
		configureBindings();
	}

	public Command getAutonomousCommand() {
		return runAuto;
	}
}
