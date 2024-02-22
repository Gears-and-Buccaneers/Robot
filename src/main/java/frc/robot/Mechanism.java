package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Mechanism implements Subsystem {
	private final CommandXboxController driver, operator; // My joystick

	private DoubleSubscriber intakeSpeed = NetworkTableInstance.getDefault().getDoubleTopic("intakeSpeed")
			.subscribe(0.6);

	private DoubleSubscriber feederSpeed = NetworkTableInstance.getDefault().getDoubleTopic("feederSpeed")
			.subscribe(-0.25);

	private DoubleSubscriber shooterSpeed = NetworkTableInstance.getDefault().getDoubleTopic("shooterSpeed")
			.subscribe(-1);

	private DoubleSubscriber pivotSpeed = NetworkTableInstance.getDefault().getDoubleTopic("pivotSpeed")
			.subscribe(0.15);

	private boolean isSetpoint = false;

	private TalonSRX intake;
	private TalonSRX intake2;

	private TalonSRX feeder;
	private TalonSRX shooter;
	private TalonFX pivot;

	@SuppressWarnings("resource")
	public Mechanism(CommandXboxController driver, CommandXboxController operator) {
		register();

		this.driver = driver;
		this.operator = operator;

		intake = new TalonSRX(9);
		intake2 = new TalonSRX(10);

		feeder = new TalonSRX(30);

		shooter = new TalonSRX(6);
		new TalonSRX(20).set(TalonSRXControlMode.Follower, 6);

		Slot0Configs conf = new Slot0Configs();

		conf.kP = 0.1 * 1;
		conf.kD = 0.000004 * 0.01 * 0;

		pivot = new TalonFX(7);
		TalonFX pivot2 = new TalonFX(17);

		pivot.getConfigurator().apply(conf);
		pivot2.getConfigurator().apply(conf);

		pivot2.setControl(new Follower(7, true));

		operator.leftTrigger().onTrue(shooter(1)).onFalse(shooter(0));
		operator.rightTrigger().onTrue(shooter(-1)).onFalse(shooter(0));

		operator.a().onTrue(feeder(1)).onFalse(feeder(0));
		operator.b().onTrue(feeder(1).alongWith(intake(1))).onFalse(feeder(0).alongWith(intake(0)));
		operator.x().onTrue(feeder(-1)).onFalse(feeder(0));
		operator.y().onTrue(intake(-1)).onFalse(intake(0));

		operator.start().onTrue(new InstantCommand(() -> pivot.setPosition(0)));

		operator.povDown().onTrue(pivot(0)).onFalse(pivotStop());
		operator.povLeft().onTrue(pivot(-3.284 * 2)).onFalse(pivotStop());
	}

	Command pivot(double pos) {
		return new InstantCommand(() -> {
			isSetpoint = true;
			pivot.setControl(new PositionDutyCycle(pos));
		});
	}

	Command pivotStop() {
		return new InstantCommand(() -> {
			isSetpoint = false;
			pivot.set(0);
		});
	}

	/**
	 * Sign should be -1, 0, or 1.
	 */
	Command feeder(int sign) {
		return new InstantCommand(() -> feeder.set(TalonSRXControlMode.PercentOutput, sign * feederSpeed.get()));
	}

	/**
	 * Sign should be -1, 0, or 1.
	 */
	Command intake(int sign) {
		return new InstantCommand(() -> {
			double speed = intakeSpeed.get() * sign;

			intake.set(TalonSRXControlMode.PercentOutput, speed);
			intake2.set(TalonSRXControlMode.PercentOutput, -speed);
		});
	}

	/**
	 * Sign should be -1, 0, or 1.
	 */
	public Command shooter(int sign) {
		return new InstantCommand(() -> shooter.set(TalonSRXControlMode.PercentOutput, sign * shooterSpeed.get()));
	}

	@Override
	public void periodic() {
		if (!isSetpoint)
			pivot.set(operator.getLeftY() * pivotSpeed.get());
		 DoubleSubscriber intakeSpeed = NetworkTableInstance.getDefault().getDoubleTopic("intakeSpeed")
			.subscribe(0.6);

		DoubleSubscriber feederSpeed = NetworkTableInstance.getDefault().getDoubleTopic("feederSpeed")
			.subscribe(-0.25);

		DoubleSubscriber shooterSpeed = NetworkTableInstance.getDefault().getDoubleTopic("shooterSpeed")
			.subscribe(-1);

		DoubleSubscriber pivotSpeed = NetworkTableInstance.getDefault().getDoubleTopic("pivotSpeed")
			.subscribe(0.15);
	}
}
