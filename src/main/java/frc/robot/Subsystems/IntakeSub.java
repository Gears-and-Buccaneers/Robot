package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeSub implements SubsystemReq {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonSRX intake1;
    private TalonSRX intake2;

    // Vars
    private double maxIntakeSpeed = 0.60;
    private String maxIntakeSpeedKey = simpleName + "Speed";
    private BooleanSupplier feederHasNote;

    public IntakeSub(BooleanSupplier feederNote) {
        this.feederHasNote = feederNote;

        // Motors
        intake1 = new TalonSRX(9);
        intake2 = new TalonSRX(10);

        intake1.setNeutralMode(NeutralMode.Coast);
        intake2.setNeutralMode(NeutralMode.Coast);
        // TODO: CurrentLimit

        // Vars
        Preferences.initDouble(maxIntakeSpeedKey, maxIntakeSpeed);

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + intake1.getClass().getSimpleName() + " ID:" + intake1.getDeviceID());
        System.out.println("\t" + intake2.getClass().getSimpleName() + " ID:" + intake2.getDeviceID());
    }

    private void runForward(boolean forwards) {
        double speed = forwards ? maxIntakeSpeed : -maxIntakeSpeed;

        intake1.set(TalonSRXControlMode.PercentOutput, speed);
        intake2.set(TalonSRXControlMode.PercentOutput, -speed);
    }

    public void disable() {
        intake1.set(TalonSRXControlMode.PercentOutput, 0);
        intake2.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public Command intake(boolean forwards) {
        return new FunctionalCommand(() -> {
            runForward(true);
        }, () -> {
        }, (_interrupted) -> {
            disable();
        }, feederHasNote);
    }

    public void loadPreferences() {
        maxIntakeSpeed = Preferences.getDouble(maxIntakeSpeedKey, maxIntakeSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringArrayProperty("ControlMode",
                () -> new String[] { intake1.getControlMode().toString(), intake2.getControlMode().toString() },
                null);
        builder.addIntegerArrayProperty("DeviceID",
                () -> new long[] { intake1.getDeviceID(), intake2.getDeviceID() }, null);

        builder.addDoubleArrayProperty("Temp",
                () -> new double[] { intake1.getTemperature(), intake2.getTemperature() }, null);
        builder.addDoubleArrayProperty("Supply Current",
                () -> new double[] { intake1.getSupplyCurrent(), intake2.getSupplyCurrent() }, null);
        builder.addDoubleArrayProperty("Stator Current",
                () -> new double[] { intake1.getStatorCurrent(), intake2.getStatorCurrent() }, null);
        builder.addDoubleArrayProperty("Output Voltage",
                () -> new double[] { intake1.getMotorOutputVoltage(), intake2.getMotorOutputVoltage() }, null);
        builder.addDoubleArrayProperty("Bus Voltage",
                () -> new double[] { intake1.getBusVoltage(), intake2.getBusVoltage() }, null);
    }
}
