package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class FeederSub implements SubsystemReq {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonSRX feeder;
    private DigitalInput limitSwitch;

    // Hardware
    private double maxFeederSpeed = 0.60;
    private String maxFeederSpeedKey = simpleName + "Speed";

    public FeederSub() {
        // Motors
        feeder = new TalonSRX(30);

        feeder.setNeutralMode(NeutralMode.Coast);

        // TODO: CurrentLimit

        // Sensors
        limitSwitch = new DigitalInput(1);

        // Vars
        Preferences.initDouble(maxFeederSpeedKey, maxFeederSpeed);

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + feeder.getClass().getSimpleName() + " ID:" + feeder.getDeviceID());
        System.out.println("[init] new limit switch on port " + 1);

    }

    private void runForward(boolean forwards) {
        double speed = forwards ? maxFeederSpeed : -maxFeederSpeed;

        feeder.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void disable() {
        feeder.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public boolean hasNote() {
        return limitSwitch.get();
    }

    public Command feed(boolean forwards) {
        return new FunctionalCommand(() -> {
            runForward(forwards);
        }, () -> {
        }, (_interrupted) -> {
            disable();
        }, () -> !hasNote());
    }

    public void loadPreferences() {
        maxFeederSpeed = Preferences.getDouble(maxFeederSpeedKey, maxFeederSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringArrayProperty("ControlMode",
                () -> new String[] { feeder.getControlMode().toString() },
                null);
        builder.addIntegerArrayProperty("DeviceID",
                () -> new long[] { feeder.getDeviceID() }, null);

        builder.addDoubleArrayProperty("Temp",
                () -> new double[] { feeder.getTemperature() }, null);
        builder.addDoubleArrayProperty("Supply Current",
                () -> new double[] { feeder.getSupplyCurrent() }, null);
        builder.addDoubleArrayProperty("Stator Current",
                () -> new double[] { feeder.getStatorCurrent() }, null);
        builder.addDoubleArrayProperty("Output Voltage",
                () -> new double[] { feeder.getMotorOutputVoltage() }, null);
        builder.addDoubleArrayProperty("Bus Voltage",
                () -> new double[] { feeder.getBusVoltage() }, null);
    }
}
