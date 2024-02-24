package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Mechanism implements Subsystem {
	// private boolean homed = false;
	private final DigitalInput limit = new DigitalInput(0);

	private final CommandXboxController operator; // My joystick

	private DoubleSubscriber intakeSpeed = NetworkTableInstance.getDefault().getDoubleTopic("intakeSpeed")
			.subscribe(0.6);

	private DoubleSubscriber feederSpeed = NetworkTableInstance.getDefault().getDoubleTopic("feederSpeed")
			.subscribe(-0.25);

	private DoubleSubscriber shooterSpeed = NetworkTableInstance.getDefault().getDoubleTopic("shooterSpeed")
			.subscribe(-1);

	private DoubleSubscriber pivotSpeed = NetworkTableInstance.getDefault().getDoubleTopic("pivotSpeed")
			.subscribe(0.15);

	private double setpoint = -1;

	private TalonSRX intake;
	private TalonSRX intake2;

	private TalonSRX feeder;
	private TalonSRX shooter;
	private TalonFX pivot;

	@SuppressWarnings("resource")
	public Mechanism(CommandXboxController driver, CommandXboxController operator) {
		register();

		this.operator = operator;

		intake = new TalonSRX(9);
		intake2 = new TalonSRX(10);

		feeder = new TalonSRX(30);

		shooter = new TalonSRX(6);
		new TalonSRX(20).set(TalonSRXControlMode.Follower, 6);

		TalonFXConfiguration conf = new TalonFXConfiguration();

		conf.Slot0.kP = 0.1 * 1;
		conf.Slot0.kD = 0.000004 * 0.01 * 0;

		conf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		conf.Feedback.SensorToMechanismRatio = 100;

		pivot = new TalonFX(7);
		TalonFX pivot2 = new TalonFX(17);

		pivot.getConfigurator().apply(conf);
		pivot2.getConfigurator().apply(conf);

		pivot2.setControl(new Follower(7, true));

		operator.leftTrigger().onTrue(shooter(1)).onFalse(shooter(0));
		operator.rightTrigger().onTrue(shooter(-1)).onFalse(shooter(0));

		operator.a().onTrue(feeder(1)).onFalse(feeder(0));
		driver.a().onTrue(feeder(1).alongWith(intake(1))).onFalse(feeder(0).alongWith(intake(0)));
		operator.b().onTrue(feeder(1).alongWith(intake(1))).onFalse(feeder(0).alongWith(intake(0)));
		operator.x().onTrue(feeder(-1)).onFalse(feeder(0));
		operator.y().onTrue(intake(-1)).onFalse(intake(0));

		operator.start().onTrue(new InstantCommand(() -> pivot.setPosition(0)));

		operator.povDown().onTrue(pivot(0)).onFalse(pivotStop());
		operator.povLeft().onTrue(pivot(0.1)).onFalse(pivotStop());
	}

	Command pivot(double pos) {
		return new InstantCommand(() -> {
			setpoint = pos;
			pivot.setControl(new PositionDutyCycle(pos));
		});
		// .onlyIf(() -> homed);
	}

	Command pivotStop() {
		return new InstantCommand(() -> {
			setpoint = -1;
			pivot.set(0);
		});
		// .onlyIf(() -> homed);
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
		// if (!homed && limit.get()) {
		// homed = true;
		// pivot.set(0);
		// pivot.setPosition(0);
		// } else if (homed && !isSetpoint)
		// pivot.set(operator.getLeftY() * pivotSpeed.get());

		double pivotS = setpoint == -1 ? operator.getLeftY() * pivotSpeed.get() : setpoint == 0 ? -1 : 1;

		if ((pivotS < 0 && !limit.get()) || (pivotS > 0 && pivot.getPosition().getValueAsDouble() < 120 / 360))
			if (setpoint == -1)
				pivot.set(pivotS);
			else
				pivot.set(0);

		if (limit.get())
			pivot.setPosition(0);

		SmartDashboard.putBoolean("Limit switch active", limit.get());
		SmartDashboard.putNumber("pivot Pose", pivot.getPosition().asSupplier().get());
		SmartDashboard.putNumber("pivot Velocity", pivot.getVelocity().asSupplier().get());
		SmartDashboard.putNumber("pivot Acceleration", pivot.getAcceleration().asSupplier().get());
		SmartDashboard.putNumber("pivot Current", pivot.getSupplyCurrent().asSupplier().get());
		SmartDashboard.putNumber("pivot Volts", pivot.getSupplyVoltage().asSupplier().get());
		SmartDashboard.putNumber("pivot Temp", pivot.getDeviceTemp().asSupplier().get());
		piviott.setAngle(pivot.getPosition().getValueAsDouble() * 3.6);
		SmartDashboard.putData("Piviot Mech", mech);
	}

	// units are inches
	private Mechanism2d mech = new Mechanism2d(28, 28);
	private MechanismRoot2d root = mech.getRoot("PivPoint", 20, 18);
	private MechanismLigament2d piviott = root.append(new MechanismLigament2d("Shooter", 18, 180));
}
