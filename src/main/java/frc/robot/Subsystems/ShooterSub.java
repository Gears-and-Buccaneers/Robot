package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShooterSub {

    private DoubleSubscriber shooterSpeed = NetworkTableInstance.getDefault().getDoubleTopic("shooterSpeed")
            .subscribe(-1);

    private TalonSRX shooter;

    @SuppressWarnings("resource")
    public ShooterSub(CommandXboxController driver, CommandXboxController operator) {

        shooter = new TalonSRX(6);
        new TalonSRX(20).set(TalonSRXControlMode.Follower, 6);
    }

    Command shooter(boolean forwards) {
        return new FunctionalCommand(() -> {
            shooter.set(TalonSRXControlMode.PercentOutput, forwards ? shooterSpeed.get() : -shooterSpeed.get());
        }, () -> {
        }, (_interrupted) -> {
            shooter.set(TalonSRXControlMode.PercentOutput, 0);
        }, () -> false);
    }

    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonSRX intake;
    private TalonSRX intake2;

    // Vars
    private double maxIntakeSpeed = 0.60;
    private String maxIntakeSpeedKey = simpleName + "Speed";

    public IntakeSub() {
        // Motors
        intake = new TalonSRX(9);
        intake2 = new TalonSRX(10);

        intake.setNeutralMode(NeutralMode.Coast);
        intake2.setNeutralMode(NeutralMode.Coast);

        // Vars
        Preferences.initDouble(maxIntakeSpeedKey, maxIntakeSpeed);

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + intake.getClass().getSimpleName() + " ID:" + intake.getBaseID());
        System.out.println("\t" + intake2.getClass().getSimpleName() + " ID:" + intake2.getBaseID());
    }

    private void runForward(boolean forwards) {
        double speed = forwards ? maxIntakeSpeed : -maxIntakeSpeed;

        intake.set(TalonSRXControlMode.PercentOutput, speed);
        intake2.set(TalonSRXControlMode.PercentOutput, -speed);
    }

    public void disable() {
        intake.set(TalonSRXControlMode.PercentOutput, 0);
        intake2.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public Command intake(boolean forwards) {
        return new FunctionalCommand(() -> {
            runForward(true);
        }, () -> {
        }, (_interrupted) -> {
            disable();
        }, () -> false);
    }

    public void loadPreferences() {
        maxIntakeSpeed = Preferences.getDouble(maxIntakeSpeedKey, maxIntakeSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }
}
