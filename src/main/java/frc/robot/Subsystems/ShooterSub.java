package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class ShooterSub implements SubsystemReq {
    private final String simpleName = this.getClass().getSimpleName();

    // Hardware
    private TalonSRX shooter1;
    private TalonSRX shooter2;

    // Vars
    private double maxShootingSpeed = 0.60;
    private String maxShootingSpeedKey = simpleName + "Speed";

    public ShooterSub() {
        // Motors
        shooter1 = new TalonSRX(9);
        shooter2 = new TalonSRX(10);

        shooter1.setNeutralMode(NeutralMode.Coast);
        shooter2.setNeutralMode(NeutralMode.Coast);

        // shooter1.configSupplyCurrentLimit(
        // new SupplyCurrentLimitConfiguration(true, 40, 100, 0.5));
        // shooter2.configSupplyCurrentLimit(
        // new SupplyCurrentLimitConfiguration(true, 40, 100, 0.5));
        // TODO: CurrentLimit

        // Vars
        Preferences.initDouble(maxShootingSpeedKey, maxShootingSpeed);

        System.out.println("[Init] Creating " + simpleName + " with:");
        System.out.println("\t" + shooter1.getClass().getSimpleName() + " ID:" + shooter1.getDeviceID());
        System.out.println("\t" + shooter2.getClass().getSimpleName() + " ID:" + shooter2.getDeviceID());
    }

    private void runForward(boolean forwards) {
        double speed = forwards ? maxShootingSpeed : -maxShootingSpeed;

        shooter1.set(TalonSRXControlMode.PercentOutput, speed);
        shooter2.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void disable() {
        shooter1.set(TalonSRXControlMode.PercentOutput, 0);
        shooter2.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public Command shoot(boolean forwards) {
        return new FunctionalCommand(() -> {
            runForward(true);
        }, () -> {
        }, (_interrupted) -> {
            disable();
        }, () -> false);
    }

    public void loadPreferences() {
        maxShootingSpeed = Preferences.getDouble(maxShootingSpeedKey, maxShootingSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.addStringArrayProperty("ControlMode",
                () -> new String[] { shooter1.getControlMode().toString(), shooter2.getControlMode().toString() },
                null);
        builder.addIntegerArrayProperty("DeviceID",
                () -> new long[] { shooter1.getDeviceID(), shooter2.getDeviceID() }, null);

        builder.addDoubleArrayProperty("Temp",
                () -> new double[] { shooter1.getTemperature(), shooter2.getTemperature() }, null);
        builder.addDoubleArrayProperty("Supply Current",
                () -> new double[] { shooter1.getSupplyCurrent(), shooter2.getSupplyCurrent() }, null);
        builder.addDoubleArrayProperty("Stator Current",
                () -> new double[] { shooter1.getStatorCurrent(), shooter2.getStatorCurrent() }, null);
        builder.addDoubleArrayProperty("Output Voltage",
                () -> new double[] { shooter1.getMotorOutputVoltage(), shooter2.getMotorOutputVoltage() }, null);
        builder.addDoubleArrayProperty("Bus Voltage",
                () -> new double[] { shooter1.getBusVoltage(), shooter2.getBusVoltage() }, null);
    }
}
